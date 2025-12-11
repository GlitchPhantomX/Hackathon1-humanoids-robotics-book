import React from 'react';

const LearningJourney = () => {
  const stages = [
    {
      number: "01",
      title: "Simulation Basics",
      description: "Master ROS 2 fundamentals, Gazebo environments, and basic robot modeling to launch your first simulations.",
      outcome: "First robot simulation running",
      weeks: "Weeks 1-3",
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10.325 4.317c.426-1.756 2.924-1.756 3.35 0a1.724 1.724 0 002.573 1.066c1.543-.94 3.31.826 2.37 2.37a1.724 1.724 0 001.065 2.572c1.756.426 1.756 2.924 0 3.35a1.724 1.724 0 00-1.066 2.573c.94 1.543-.826 3.31-2.37 2.37a1.724 1.724 0 00-2.572 1.065c-.426 1.756-2.924 1.756-3.35 0a1.724 1.724 0 00-2.573-1.066c-1.543.94-3.31-.826-2.37-2.37a1.724 1.724 0 00-1.065-2.572c-1.756-.426-1.756-2.924 0-3.35a1.724 1.724 0 001.066-2.573c-.94-1.543.826-3.31 2.37-2.37.996.608 2.296.07 2.572-1.065z" />
        </svg>
      )
    },
    {
      number: "02",
      title: "Digital Twin Mastery",
      description: "Create high-fidelity digital twins with Unity/Gazebo, simulate complex sensors, and build realistic physics environments.",
      outcome: "Complex environments & sensor integration",
      weeks: "Weeks 4-6",
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19.428 15.428a2 2 0 00-1.022-.547l-2.387-.477a6 6 0 00-3.86.517l-.318.158a6 6 0 01-3.86.517L6.05 15.21a2 2 0 00-1.806.547M8 4h8l-1 1v5.172a2 2 0 00.586 1.414l5 5c1.26 1.26.367 3.414-1.415 3.414H4.828c-1.782 0-2.674-2.154-1.414-3.414l5-5A2 2 0 009 10.172V5L8 4z" />
        </svg>
      )
    },
    {
      number: "03",
      title: "AI Perception & Navigation",
      description: "Implement NVIDIA Isaac, SLAM, computer vision, and Nav2 for autonomous navigation in complex environments.",
      outcome: "AI-powered robot perception",
      weeks: "Weeks 7-10",
      popular: true,
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 12a3 3 0 11-6 0 3 3 0 016 0z" />
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M2.458 12C3.732 7.943 7.523 5 12 5c4.478 0 8.268 2.943 9.542 7-1.274 4.057-5.064 7-9.542 7-4.477 0-8.268-2.943-9.542-7z" />
        </svg>
      )
    },
    {
      number: "04",
      title: "Voice-Language-Action",
      description: "Integrate GPT models and Whisper to enable natural language commands that translate into executable robot behaviors.",
      outcome: "Conversational humanoid robots",
      weeks: "Weeks 11-12",
      popular: true,
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M19 11a7 7 0 01-7 7m0 0a7 7 0 01-7-7m7 7v4m0 0H8m4 0h4m-4-8a3 3 0 01-3-3V5a3 3 0 116 0v6a3 3 0 01-3 3z" />
        </svg>
      )
    },
    {
      number: "05",
      title: "Hardware Deployment",
      description: "Deploy trained models to Jetson hardware, control real robots, and bring your AI-powered systems to life.",
      outcome: "Autonomous physical robots",
      weeks: "Week 13+",
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
        </svg>
      )
    },
    {
      number: "06",
      title: "Advanced Integration",
      description: "Combine all learned concepts to build complete robotic systems with multi-modal AI capabilities.",
      outcome: "Production-ready robot systems",
      weeks: "Ongoing",
      icon: (
        <svg className="w-8 h-8" fill="none" stroke="currentColor" viewBox="0 0 24 24">
          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
        </svg>
      )
    }
  ];

  return (
    <section className="pb-16 pt-7 bg-[var(--color-bg)]">
      <div className="container mx-auto px-4">
        {/* Header */}
        <div className="text-center mb-16 animate-fadeIn">
          <div className="inline-flex items-center gap-2 text-[12px] text-orange-800 bg-[var(--color-badge-background-top)] border border-orange-200 rounded-full px-4 py-1 shadow-sm mb-4">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M9 20l-5.447-2.724A1 1 0 013 16.382V5.618a1 1 0 011.447-.894L9 7m0 13l6-3m-6 3V7m6 10l4.553 2.276A1 1 0 0021 18.382V7.618a1 1 0 00-.553-.894L15 4m0 13V4m0 0L9 7" stroke="#f97316" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
            <span>Master Physical AI Step by Step</span>
          </div>
          
          <h2 className="text-3xl md:text-4xl font-bold text-[var(--color-text)] mb-4 animate-slideUp">
            Your Robotics Learning Journey
          </h2>
          
          <p className="text-[15px] text-[var(--color-description-text)] max-w-2xl mx-auto animate-fadeIn delay-200">
            Progress from simulation fundamentals to deploying intelligent robots in the real world. Each milestone builds your expertise systematically.
          </p>
        </div>

        {/* Journey Cards */}
        <div className="relative max-w-6xl mx-auto">
          {/* Connection Line - Desktop */}
          {/* <div className="hidden lg:block absolute top-24 left-0 right-0 h-1 bg-gradient-to-r from-orange-200 via-orange-300 to-orange-200 rounded-full" style={{top: '120px'}}></div> */}
          
          {/* Cards Grid */}
          <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-8 relative">
            {stages.map((stage, index) => (
              <div 
                key={index} 
                className="group relative animate-fadeIn"
                style={{animationDelay: `${index * 100}ms`}}
              >
                {/* Popular Badge */}
                {stage.popular && (
                  <div className="absolute -top-2 -right-2 bg-gradient-to-r from-orange-500 to-orange-600 text-white text-[9px] font-bold px-3 py-1 rounded-full shadow-lg z-20 rotate-12">
                    POPULAR
                  </div>
                )}

                {/* Card */}
                <div className="relative bg-[var(--color-bg)] border-[1px] shadow-lg shadow-[var(--color-learning-journey-card-shadow-color)] border-orange-200 rounded-2xl p-6 transition-all duration-500 hover:shadow-2xl cursor-pointer h-full flex flex-col">
                  

                  {/* Icon */}
                  <div className="mt-6 mb-4 flex justify-center">
                    <div className="w-16 h-16 bg-orange-50 rounded-xl flex items-center justify-center text-orange-600 group-hover:bg-orange-100 group-hover:scale-110 transition-all duration-500">
                      {stage.icon}
                    </div>
                  </div>

                  {/* Content */}
                  <div className="flex-1 flex flex-col">
                    <h3 className="text-lg font-bold text-[var(--color-text)] mb-3 text-center group-hover:text-orange-600 transition-colors duration-500">
                      {stage.title}
                    </h3>
                    
                    <p className="text-[13px] text-[var(--color-card-description-text)] mb-4 text-center leading-relaxed flex-1">
                      {stage.description}
                    </p>

                    {/* Outcome Badge */}
                    <div className="bg-orange-50 border border-orange-100 rounded-lg p-2 mb-3">
                      <p className="text-[11px] text-orange-700 font-semibold text-center">
                        ✓ {stage.outcome}
                      </p>
                    </div>

                    {/* Duration */}
                    <div className="text-center">
                      <span className="inline-block bg-gray-100 text-gray-700 text-[11px] font-medium px-3 py-1 rounded-full group-hover:bg-orange-100 group-hover:text-orange-700 transition-colors duration-500">
                        {stage.weeks}
                      </span>
                    </div>
                  </div>

                  {/* Bottom Progress Line */}
                  {/* <div className="absolute bottom-0 left-0 right-0 h-1 bg-orange-100 overflow-hidden" style={{borderBottomLeftRadius: '1rem', borderBottomRightRadius: '1rem'}}>
                    <div className="h-full bg-gradient-to-r from-orange-400 to-orange-600 w-0 group-hover:w-full transition-all duration-700 ease-out"></div>
                  </div> */}

                  {/* Hover Glow Effect */}
                  {/* <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-orange-50/20 group-hover:to-orange-100/10 rounded-2xl transition-all duration-500 pointer-events-none"></div> */}
                </div>
              </div>
            ))}
          </div>
        </div>
      </div>

      <style>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(20px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(30px); }
          to { opacity: 1; transform: translateY(0); }
        }
        .animate-fadeIn {
          animation: fadeIn 0.6s ease-out forwards;
          opacity: 0;
        }
        .animate-slideUp {
          animation: slideUp 0.7s ease-out forwards;
          opacity: 0;
        }
        .delay-200 { animation-delay: 0.2s; }
      `}</style>
    </section>
  );
};

export default LearningJourney;