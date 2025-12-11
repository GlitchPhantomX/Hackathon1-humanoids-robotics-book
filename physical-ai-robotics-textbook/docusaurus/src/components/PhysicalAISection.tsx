import React from "react";

export default function PhysicalAISection() {
  return (
    <section className="py-14 text-gray-800 dark:text-white">
      <div className="container mx-auto px-4">
        <div className="flex items-center justify-center mb-6 animate-fadeIn">
          <div className="flex items-center gap-2 text-[12px] text-orange-800 dark:text-white bg-[var(--color-badge-background-top)] border border-orange-200 dark:border-orange-300 rounded-full px-4 py-1 shadow-sm">
            <svg
              width="14"
              height="8"
              viewBox="0 0 14 8"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M9.4 1H13v3.6"
                stroke="#f97316"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              <path
                d="M13 1 7.9 6.1l-3-3L1 7"
                stroke="#f97316"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>Physical AI & Robotics Course – 2025 Edition</span>
          </div>
        </div>

        <h2 className="text-3xl md:text-4xl font-bold text-center mb-6 text-[var(--color-text)] tracking-tight animate-slideUp">
          Physical AI & Humanoid Robotics
        </h2>

        <p className="text-center text-[15px] max-w-3xl mx-auto mb-14 text-[var(--color-description-text)] dark:text-gray-300 animate-fadeIn delay-200">
          Explore how embodied intelligence connects digital AI with real-world robotic systems capable of perception, action, and natural interaction.
        </p>

        <div className="grid grid-cols-1 md:grid-cols-3 gap-8">
          {/* Card 1 */}
          <div className="bg-[var(--color-bg)] border border-orange-100 dark:border-gray-700 rounded-xl p-6 shadow-md hover:shadow-xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-200 hover:border-orange-300 dark:hover:border-orange-500">
            <h3 className="text-[19px] font-semibold text-orange-600 dark:text-orange-400 text-center">
              Embodied Intelligence
            </h3>
            <p className="text-gray-500 dark:text-gray-300 text-[12px] text-center mb-4">
              AI grounded in physical reality
            </p>

            <p className="text-[var(--color-card-description-text)] dark:text-gray-200 mb-4 text-[14px] text-center">
              Learn how robots sense and react to real-world physics, enabling movements and decisions rooted in the physical environment.
            </p>

            <ul className="space-y-2 mb-4">
              {[
                "Physical AI foundations",
                "Physics-aware control",
                "Real-world interaction",
              ].map((item, i) => (
                <li key={i} className="flex items-start gap-2 text-[var(--color-card-description-text)] text-sm">
                  <span className="mt-[5px] h-2 w-2 bg-orange-500 rounded-sm rotate-45"></span>
                  {item}
                </li>
              ))}
            </ul>

            <div className="bg-[var(--color-background-card-example)] border-l-4 border-orange-400 p-2.5 rounded-md text-gray-600 dark:text-gray-300 text-xs italic shadow-sm">
              Example: A robot adjusting grip for different shapes.
            </div>
          </div>

          {/* Card 2 */}
          <div className="bg-[var(--color-bg)] border border-orange-100 dark:border-gray-700 rounded-xl p-6 shadow-md hover:shadow-xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-300 hover:border-orange-300 dark:hover:border-orange-500">
            <h3 className="text-[19px] font-semibold text-orange-600 dark:text-orange-400 text-center">
              Digital Twins & Simulation
            </h3>
            <p className="text-gray-500 dark:text-gray-300 text-[12px] text-center mb-4">
              Virtual robotics testing
            </p>

            <p className="text-[var(--color-card-description-text)] mb-4 text-[14px] text-center">
              Build high-fidelity digital twins with Gazebo and Unity to safely test robot behavior before deploying to real hardware.
            </p>

            <ul className="space-y-2 mb-4">
              {[
                "Realistic simulations",
                "Sensor-accurate testing",
                "Complex virtual scenes",
              ].map((item, i) => (
                <li key={i} className="flex items-start gap-2 text-[var(--color-card-description-text)] text-sm">
                  <span className="mt-[5px] h-2 w-2 bg-orange-500 rounded-sm rotate-45"></span>
                  {item}
                </li>
              ))}
            </ul>

            <div className="bg-orange-50 dark:bg-gray-800 border-l-4 border-orange-400 p-2.5 rounded-md text-gray-600 dark:text-gray-300 text-xs italic shadow-sm">
              Example: A humanoid navigating a virtual worksite.
            </div>
          </div>

          {/* Card 3 */}
          <div className="bg-[var(--color-bg)] border border-orange-100 dark:border-gray-700 rounded-xl p-6 shadow-md hover:shadow-xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-400 hover:border-orange-300 dark:hover:border-orange-500">
            <h3 className="text-[19px] font-semibold text-orange-600 dark:text-orange-400 text-center">
              AI-Robot Brain
            </h3>
            <p className="text-gray-500 dark:text-gray-300 text-[12px] text-center mb-4">
              Perception & decision-making
            </p>

            <p className="text-[var(--color-card-description-text)] mb-4 text-[14px] text-center">
              Use NVIDIA Isaac to power robot perception, navigation, and language-driven decision-making for autonomous interactions.
            </p>

            <ul className="space-y-2 mb-4">
              {[
                "Visual understanding",
                "SLAM & planning",
                "VLA intelligence",
              ].map((item, i) => (
                <li key={i} className="flex items-start gap-2 text-[var(--color-card-description-text)] text-sm">
                  <span className="mt-[5px] h-2 w-2 bg-orange-500 rounded-sm rotate-45"></span>
                  {item}
                </li>
              ))}
            </ul>

            <div className="bg-orange-50 dark:bg-gray-800 border-l-4 border-orange-400 p-2.5 rounded-md text-gray-600 dark:text-gray-300 text-xs italic shadow-sm">
              Example: A robot executing a spoken command.
            </div>
          </div>
        </div>
      </div>

      <style>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(10px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(25px); }
          to { opacity: 1; transform: translateY(0); }
        }
        .animate-fadeIn { animation: fadeIn 0.6s ease forwards; }
        .animate-slideUp { animation: slideUp 0.7s ease forwards; }
        .delay-200 { animation-delay: 0.2s; }
        .delay-300 { animation-delay: 0.3s; }
        .delay-400 { animation-delay: 0.4s; }
      `}</style>
    </section>
  );
}
