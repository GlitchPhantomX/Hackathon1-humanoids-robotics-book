import React, { useState } from 'react';

interface LearningStage {
  title: string;
  description: string;
  outcome: string;
  badge: string;
  isMostPopular?: boolean;
  icon: string; // SVG path data for the icon
}

interface LearningJourneyCardProps {
  stage: LearningStage;
  stageNumber: number;
  delay: number; // for fade-in animation
}

// Placeholder SVG Icon component - using a simple path for demonstration
const SvgIcon = ({ iconPath, className }: { iconPath: string; className: string }) => (
  <svg
    className={className}
    xmlns="http://www.w3.org/2000/svg"
    fill="none"
    viewBox="0 0 24 24"
    stroke="currentColor"
    strokeWidth="2"
  >
    <path strokeLinecap="round" strokeLinejoin="round" d={iconPath} />
  </svg>
);

const LearningJourneyCard: React.FC<LearningJourneyCardProps> = ({ stage, stageNumber, delay }) => {
  const [isHovered, setIsHovered] = useState(false);

  const cardClasses = `
    relative bg-white p-6 rounded-lg shadow-md border border-gray-200 transition-all duration-300 ease-in-out
    flex flex-col items-center text-center
    ${isHovered ? 'transform -translate-y-2 shadow-lg scale-[1.02]' : ''}
    learning-journey-card-fade-in
  `;

  const iconContainerClasses = `
    w-16 h-16 mb-4 p-3 rounded-full flex items-center justify-center
    transition-all duration-300 ease-in-out
    ${isHovered ? 'scale-110 bg-gradient-to-br from-orange-400 to-orange-600 text-white' : 'bg-orange-100 text-orange-500'}
  `;

  const titleClasses = `
    text-xl font-bold mb-2 transition-colors duration-300 ease-in-out
    ${isHovered ? 'text-white' : 'text-orange-500'}
  `;

  const descriptionClasses = `
    text-gray-700 mb-4
    ${isHovered ? 'text-gray-200' : ''}
  `;

  const outcomeClasses = `
    text-sm font-semibold text-gray-600 italic mb-4
    ${isHovered ? 'text-gray-300' : ''}
  `;

  const badgeClasses = `
    absolute -top-3 -left-3 bg-orange-500 text-white text-xs font-bold px-3 py-1 rounded-full shadow-md z-20
  `;

  const durationBadgeClasses = `
    mt-auto bg-gray-100 text-gray-700 text-xs font-semibold px-2 py-1 rounded
    ${isHovered ? 'bg-gray-700 text-gray-100' : ''}
  `;

  const popularTagClasses = `
    absolute -top-2 right-[-2.5rem] bg-orange-500 text-white text-xs font-bold px-6 py-1 rounded-full
    transform rotate-45 origin-bottom-left shadow-md z-20
  `;

  // Custom CSS for animations and hover effects
  const customCss = `
    .learning-journey-card-fade-in {
      opacity: 0;
      animation: fadeIn 0.8s ease-out forwards;
      animation-delay: ${delay}ms;
    }

    @keyframes fadeIn {
      from {
        opacity: 0;
        transform: translateY(20px);
      }
      to {
        opacity: 1;
        transform: translateY(0);
      }
    }

    .card-bottom-line {
      position: absolute;
      bottom: 0;
      left: 0;
      height: 4px;
      width: 0%;
      background: linear-gradient(to right, #f97316, #ea580c);
      transition: width 0.3s ease-in-out;
      border-bottom-left-radius: 0.5rem; /* Match card rounded-lg */
      border-bottom-right-radius: 0.5rem; /* Match card rounded-lg */
      z-index: 1; /* Ensure it's above the card but below content */
    }

    .learning-journey-card:hover .card-bottom-line {
      width: 100%;
    }

    .learning-journey-card-gradient-overlay {
        position: absolute;
        top: 0;
        left: 0;
        width: 100%;
        height: 100%;
        border-radius: 0.5rem; /* Match card rounded-lg */
        background: linear-gradient(to bottom right, rgba(249, 115, 22, 0.8), rgba(234, 88, 12, 0.8));
        opacity: 0;
        transition: opacity 0.3s ease-in-out;
        z-index: 0; /* Behind content */
    }

    .learning-journey-card:hover .learning-journey-card-gradient-overlay {
        opacity: 1;
    }

    .learning-journey-card-content {
        position: relative;
        z-index: 10; /* Ensure content is above overlay */
    }
  `;

  return (
    <div
      className={`${cardClasses} learning-journey-card`} // Add learning-journey-card for custom CSS target
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      style={{ animationDelay: `${delay}ms` }}
    >
      {stage.isMostPopular && (
        <span className={popularTagClasses}>MOST POPULAR</span>
      )}
      <span className={badgeClasses}>{stageNumber}</span>
      <div className="learning-journey-card-gradient-overlay"></div> {/* Overlay */}
      <div className="learning-journey-card-content"> {/* Content wrapper */}
          <div className={iconContainerClasses}>
              <SvgIcon iconPath={stage.icon} className="w-8 h-8" />
          </div>
          <h3 className={titleClasses}>{stage.title}</h3>
          <p className={descriptionClasses}>{stage.description}</p>
          <p className={outcomeClasses}>Outcome: {stage.outcome}</p>
          <span className={durationBadgeClasses}>{stage.badge}</span>
      </div>
      <div className="card-bottom-line"></div>
      <style>{customCss}</style>
    </div>
  );
};

export default LearningJourneyCard;