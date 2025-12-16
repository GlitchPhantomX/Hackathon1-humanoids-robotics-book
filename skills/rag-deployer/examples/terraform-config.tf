# Terraform configuration for RAG system infrastructure

terraform {
  required_version = ">= 1.0"
  required_providers {
    aws = {
      source  = "hashicorp/aws"
      version = "~> 5.0"
    }
  }
}

# Configure AWS provider
provider "aws" {
  region = var.aws_region
}

# Variables
variable "aws_region" {
  description = "AWS region for deployment"
  type        = string
  default     = "us-west-2"
}

variable "environment" {
  description = "Environment name"
  type        = string
  default     = "production"
}

variable "instance_type" {
  description = "EC2 instance type for RAG system"
  type        = string
  default     = "t3.medium"
}

# VPC configuration
resource "aws_vpc" "rag_vpc" {
  cidr_block           = "10.0.0.0/16"
  enable_dns_hostnames = true
  enable_dns_support   = true

  tags = {
    Name = "rag-vpc-${var.environment}"
  }
}

# Subnets
resource "aws_subnet" "rag_subnet" {
  count                   = 2
  vpc_id                  = aws_vpc.rag_vpc.id
  cidr_block              = "10.0.${count.index}.0/24"
  availability_zone       = data.aws_availability_zones.available.names[count.index]
  map_public_ip_on_launch = true

  tags = {
    Name = "rag-subnet-${var.environment}-${count.index}"
  }
}

data "aws_availability_zones" "available" {
  state = "available"
}

# Internet Gateway
resource "aws_internet_gateway" "rag_igw" {
  vpc_id = aws_vpc.rag_vpc.id

  tags = {
    Name = "rag-igw-${var.environment}"
  }
}

# Route Table
resource "aws_route_table" "rag_route_table" {
  vpc_id = aws_vpc.rag_vpc.id

  route {
    cidr_block = "0.0.0.0/0"
    gateway_id = aws_internet_gateway.rag_igw.id
  }

  tags = {
    Name = "rag-route-table-${var.environment}"
  }
}

resource "aws_route_table_association" "rag_route_table_assoc" {
  count          = length(aws_subnet.rag_subnet)
  subnet_id      = aws_subnet.rag_subnet[count.index].id
  route_table_id = aws_route_table.rag_route_table.id
}

# Security Group for RAG system
resource "aws_security_group" "rag_sg" {
  name_prefix = "rag-sg-${var.environment}"
  vpc_id      = aws_vpc.rag_vpc.id

  ingress {
    description = "HTTP"
    from_port   = 80
    to_port     = 80
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    description = "HTTPS"
    from_port   = 443
    to_port     = 443
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  ingress {
    description = "Custom HTTP port"
    from_port   = 8000
    to_port     = 8000
    protocol    = "tcp"
    cidr_blocks = ["0.0.0.0/0"]
  }

  egress {
    from_port   = 0
    to_port     = 0
    protocol    = "-1"
    cidr_blocks = ["0.0.0.0/0"]
  }

  tags = {
    Name = "rag-sg-${var.environment}"
  }
}

# ECR Repository for RAG system image
resource "aws_ecr_repository" "rag_ecr" {
  name                 = "rag-system-${var.environment}"
  image_tag_mutability = "MUTABLE"

  image_scanning_configuration {
    scan_on_push = true
  }

  tags = {
    Name = "rag-ecr-${var.environment}"
  }
}

# ECS Cluster
resource "aws_ecs_cluster" "rag_cluster" {
  name = "rag-cluster-${var.environment}"

  setting {
    name  = "containerInsights"
    value = "enabled"
  }

  tags = {
    Name = "rag-cluster-${var.environment}"
  }
}

# ECS Task Definition
resource "aws_ecs_task_definition" "rag_task" {
  family                   = "rag-task-${var.environment}"
  network_mode             = "awsvpc"
  requires_compatibilities = ["FARGATE"]
  cpu                      = "512"
  memory                   = "1024"
  execution_role_arn       = aws_iam_role.ecs_execution_role.arn
  task_role_arn            = aws_iam_role.ecs_task_role.arn
  runtime_platform {
    operating_system_family = "LINUX"
  }

  container_definitions = jsonencode([
    {
      name      = "rag-container"
      image     = "${aws_ecr_repository.rag_ecr.repository_url}:latest"
      cpu       = 512
      memory    = 1024
      essential = true
      portMappings = [
        {
          containerPort = 8000
          protocol      = "tcp"
        }
      ]
      environment = [
        {
          name  = "QDRANT_URL"
          value = "http://qdrant-service:6333"
        }
      ]
      secrets = [
        {
          name      = "OPENAI_API_KEY"
          valueFrom = aws_ssm_parameter.openai_api_key.arn
        }
      ]
      logConfiguration = {
        logDriver = "awslogs"
        options = {
          awslogs-group         = aws_cloudwatch_log_group.rag_logs.name
          awslogs-region        = var.aws_region
          awslogs-stream-prefix = "ecs"
        }
      }
    }
  ])

  tags = {
    Name = "rag-task-${var.environment}"
  }
}

# ECS Service
resource "aws_ecs_service" "rag_service" {
  name            = "rag-service-${var.environment}"
  cluster         = aws_ecs_cluster.rag_cluster.id
  task_definition = aws_ecs_task_definition.rag_task.arn
  desired_count   = 3
  launch_type     = "FARGATE"

  network_configuration {
    security_groups  = [aws_security_group.rag_sg.id]
    subnets          = aws_subnet.rag_subnet[*].id
    assign_public_ip = true
  }

  load_balancer {
    target_group_arn = aws_lb_target_group.rag_target_group.arn
    container_name   = "rag-container"
    container_port   = 8000
  }

  tags = {
    Name = "rag-service-${var.environment}"
  }
}

# Application Load Balancer
resource "aws_lb" "rag_alb" {
  name               = "rag-alb-${var.environment}"
  internal           = false
  load_balancer_type = "application"
  security_groups    = [aws_security_group.rag_sg.id]
  subnets            = aws_subnet.rag_subnet[*].id

  tags = {
    Name = "rag-alb-${var.environment}"
  }
}

# ALB Listener
resource "aws_lb_listener" "rag_listener" {
  load_balancer_arn = aws_lb.rag_alb.arn
  port              = "80"
  protocol          = "HTTP"

  default_action {
    type             = "forward"
    target_group_arn = aws_lb_target_group.rag_target_group.arn
  }
}

# ALB Target Group
resource "aws_lb_target_group" "rag_target_group" {
  name     = "rag-target-group-${var.environment}"
  port     = 8000
  protocol = "HTTP"
  vpc_id   = aws_vpc.rag_vpc.id
  target_type = "ip"

  health_check {
    enabled             = true
    healthy_threshold   = 2
    unhealthy_threshold = 5
    timeout             = 5
    interval            = 30
    path                = "/health"
    matcher             = "200"
  }

  tags = {
    Name = "rag-target-group-${var.environment}"
  }
}

# IAM Roles and Policies
resource "aws_iam_role" "ecs_execution_role" {
  name = "ecs-execution-role-${var.environment}"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "ecs-tasks.amazonaws.com"
        }
      }
    ]
  })

  tags = {
    Name = "ecs-execution-role-${var.environment}"
  }
}

resource "aws_iam_role_policy_attachment" "ecs_execution_role_policy" {
  role       = aws_iam_role.ecs_execution_role.name
  policy_arn = "arn:aws:iam::aws:policy/service-role/AmazonECSTaskExecutionRolePolicy"
}

resource "aws_iam_role" "ecs_task_role" {
  name = "ecs-task-role-${var.environment}"

  assume_role_policy = jsonencode({
    Version = "2012-10-17"
    Statement = [
      {
        Action = "sts:AssumeRole"
        Effect = "Allow"
        Principal = {
          Service = "ecs-tasks.amazonaws.com"
        }
      }
    ]
  })

  tags = {
    Name = "ecs-task-role-${var.environment}"
  }
}

# CloudWatch Log Group
resource "aws_cloudwatch_log_group" "rag_logs" {
  name              = "/ecs/rag-${var.environment}"
  retention_in_days = 30

  tags = {
    Name = "rag-logs-${var.environment}"
  }
}

# SSM Parameter for API Key
resource "aws_ssm_parameter" "openai_api_key" {
  name  = "/rag/${var.environment}/openai-api-key"
  type  = "SecureString"
  value = var.openai_api_key

  tags = {
    Name = "openai-api-key-${var.environment}"
  }
}

variable "openai_api_key" {
  description = "OpenAI API Key (will be stored as secure parameter)"
  type        = string
  sensitive   = true
}

output "alb_dns_name" {
  description = "DNS name of the Application Load Balancer"
  value       = aws_lb.rag_alb.dns_name
}

output "ecr_repository_url" {
  description = "URL of the ECR repository"
  value       = aws_ecr_repository.rag_ecr.repository_url
}