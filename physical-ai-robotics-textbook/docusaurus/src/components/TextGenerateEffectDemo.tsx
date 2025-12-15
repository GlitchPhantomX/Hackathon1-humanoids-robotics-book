"use client";
import React from "react"
import { TextGenerateEffect } from "../ui/text-generate-effect";

const words = `The future of AI extends beyond digital spaces into the physical world. Physical AI systems function in reality and comprehend physical laws. Master ROS 2, NVIDIA Isaac, and humanoid robotics. Design robots capable of natural human interactions. Bridge the gap between the digital brain and the physical body. From simulation to real-world deployment.`;

export function TextGenerateEffectDemo() {
  return <TextGenerateEffect words={words} />;
}