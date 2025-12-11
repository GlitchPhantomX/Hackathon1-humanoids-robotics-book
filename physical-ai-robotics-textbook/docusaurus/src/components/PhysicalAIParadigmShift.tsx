import React from "react";
import { MovingBorderDemo } from "./MovingBorderDemo2";

export default function PhysicalAIParadigmShift() {
  return (
    <section className="py-14 bg-[var(--color-bg)]">
      <div className="container mx-auto px-4">
        {/* Badge */}
        <div className="flex items-center justify-center mb-6 animate-fadeIn">
          <div className="flex items-center gap-2 text-[12px] text-orange-800 bg-[var(--color-badge-background-top)] border border-orange-200 rounded-full px-4 py-1 shadow-sm">
            <svg
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              xmlns="http://www.w3.org/2000/svg"
            >
              <path
                d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"
                stroke="#f97316"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>The Evolution of Robotics</span>
          </div>
        </div>

        {/* Heading */}
        <h2 className="text-3xl md:text-4xl font-bold text-center mb-4 text-[var(--color-text)] tracking-tight animate-slideUp">
          The Great Shift
        </h2>

        <p className="text-center text-lg text-orange-600 font-semibold mb-3 animate-fadeIn delay-200">
          From Automation to Intelligence • From Coding to Co-Creating
        </p>

        {/* Description */}
        <p className="text-center text-[15px] max-w-3xl mx-auto mb-14 text-[var(--color-description-text)] animate-fadeIn delay-300">
          Physical AI development is not about replacing roboticists—it's about amplifying human capability. 
          Learn to collaborate with intelligent systems that understand the physical world and adapt with you.
        </p>

        {/* Comparison Cards */}
        <div className="grid grid-cols-1 lg:grid-cols-2 gap-12 max-w-6xl mx-auto relative">
          {/* Traditional Approach */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-8 shadow-sm hover:shadow-xl transition-all duration-300 animate-fadeIn delay-400 relative overflow-hidden">

            <div className="absolute inset-0 bg-gradient-to-br from-gray-50/0 to-gray-50/0 group-hover:from-[var(--color-physical-from-shift-hover-shadow-effect)] group-hover:to-[var(--color-physical-to-shift-hover-shadow-effect)] transition-all duration-300"></div>
            
            <div className="relative z-10">
              {/* Icon and Title */}
              <div className="flex items-center gap-3 mb-6">
                <div className="flex items-center justify-center w-12 h-12 bg-gray-100 rounded-lg group-hover:scale-110 transition-transform duration-300">
                  <svg className="w-6 h-6 text-gray-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253" />
                  </svg>
                </div>
                <div>
                  <h3 className="text-xl font-bold text-[var(--color-card-description-text)]">Traditional Robotics</h3>
                  <p className="text-[12px] text-gray-500 italic">The pre-programmed era</p>
                </div>
              </div>

              {/* Feature List */}
              <div className="space-y-4">
                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Instruction-Based Control</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Program robots with exact trajectories and fixed behavior trees</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Manual Calibration</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Engineer hand-tunes sensors, actuators, and kinematics</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Static Environments</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Robots operate in controlled factory settings with predictable conditions</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Linear Learning Path</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Learn mechanics → Study control theory → Build simple bots → Slowly scale</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-gray-400 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-[var(--color-card-description-text)] text-[15px] mb-1">Hardware-First Approach</p>
                    <p className="text-[var(--color-description-text)] text-[13px]">Focus on mechanical design and motor control from day one</p>
                  </div>
                </div>
              </div>
            </div>
          </div>

          {/* Physical AI Approach */}
          <div className="group bg-gradient-to-br from-[var(--color-great-shift-from-shadow-color)] to-[var(--color-great-shift-to-shadow-color)] border-2 border-orange-200 rounded-xl p-8 shadow-lg hover:shadow-2xl transition-all duration-300 animate-fadeIn delay-500 relative overflow-hidden">
            {/* Premium Badge */}
            <div className="absolute top-4 right-4 bg-gradient-to-r from-orange-500 to-orange-600 text-white text-[10px] font-bold px-3 py-1 rounded-full shadow-md z-20">
              FUTURE OF ROBOTICS
            </div>

            <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-100/0 group-hover:from-orange-50/30 group-hover:to-orange-100/20 transition-all duration-300"></div>
            
            <div className="relative z-10">
              {/* Icon and Title */}
              <div className="flex items-center gap-3 mb-6">
                <div className="flex items-center justify-center w-12 h-12 bg-orange-100 rounded-lg group-hover:scale-110 transition-transform duration-300">
                  <svg className="w-6 h-6 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                    <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                  </svg>
                </div>
                <div>
                  <h3 className="text-xl font-bold text-orange-700">Physical AI Way</h3>
                  <p className="text-[12px] text-orange-600 italic">The intelligence era</p>
                </div>
              </div>

              {/* Feature List */}
              <div className="space-y-4">
                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-orange-500 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-orange-700 text-[15px] mb-1">Intent-Based Intelligence</p>
                    <p className="text-gray-700 text-[13px]">Describe goals; AI reasons physics, plans motion, and adapts behavior</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-orange-500 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-orange-700 text-[15px] mb-1">Self-Learning Systems</p>
                    <p className="text-gray-700 text-[13px]">You and AI collaborate—robot improves through reinforcement and simulation</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-orange-500 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-orange-700 text-[15px] mb-1">Digital Twin Simulation</p>
                    <p className="text-gray-700 text-[13px]">Train in virtual worlds (Isaac Sim), deploy to real humanoids seamlessly</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-orange-500 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-orange-700 text-[15px] mb-1">Production-First Learning</p>
                    <p className="text-gray-700 text-[13px]">Build autonomous humanoids from day one using ROS 2 and NVIDIA Isaac</p>
                  </div>
                </div>

                <div className="flex gap-3 group/item">
                  <div className="flex-shrink-0 w-1.5 h-1.5 bg-orange-500 rounded-full mt-2"></div>
                  <div>
                    <p className="font-semibold text-orange-700 text-[15px] mb-1">Architecture-First Design</p>
                    <p className="text-gray-700 text-[13px]">Design intelligent perception pipelines, not just joint controllers</p>
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>

        {/* VS Divider - Desktop only */}
        <div className="hidden lg:flex relative left-1/2 top-[-520px] -translate-x-1/2 -translate-y-1/2 items-center justify-center w-16 h-16 bg-white border-4 border-orange-200 rounded-full shadow-lg z-10">
          <span className="text-orange-600 font-bold text-sm">VS</span>
        </div>

        {/* Bottom CTA */}
        <div className="mt-4 text-center animate-fadeIn delay-600 shadow-sm hover:shadow-lg transition-all duration-300">
        <MovingBorderDemo/>
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
        .animate-fadeIn { 
          animation: fadeIn 0.6s ease forwards; 
          opacity: 0;
        }
        .animate-slideUp { 
          animation: slideUp 0.7s ease forwards; 
          opacity: 0;
        }
        .delay-200 { animation-delay: 0.2s; }
        .delay-300 { animation-delay: 0.3s; }
        .delay-400 { animation-delay: 0.4s; }
        .delay-500 { animation-delay: 0.5s; }
        .delay-600 { animation-delay: 0.6s; }
      `}</style>
    </section>
  );
}