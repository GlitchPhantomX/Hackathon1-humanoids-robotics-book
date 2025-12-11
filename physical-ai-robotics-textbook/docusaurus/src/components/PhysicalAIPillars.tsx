import React from "react";
import { MovingBorderDemo } from "./MovingBorderDemo";

export default function PhysicalAIPillars() {
  return (
    <section className="py-14 text-gray-800">
      <div className="container mx-auto px-4">
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
                d="M12 2L2 7l10 5 10-5-10-5z"
                stroke="#f97316"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
              <path
                d="M2 17l10 5 10-5M2 12l10 5 10-5"
                stroke="#f97316"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              />
            </svg>
            <span>What Makes This Textbook Different</span>
          </div>
        </div>

        <h2 className="text-3xl md:text-4xl font-bold text-center mb-6 text-[var(--color-text)] tracking-tight animate-slideUp">
          Core Learning Pillars
        </h2>

        <p className="text-center text-[15px] max-w-3xl mx-auto mb-14 text-[var(--color-description-text)] animate-fadeIn delay-200">
          A comprehensive, simulation-first approach to mastering Physical AI and humanoid robotics through hands-on practice.
        </p>

        <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-6 mb-7">
          {/* Pillar 1 */}
          <div className="group bg-[var(--color-bg)] border-gray-200 rounded-xl p-6  transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-200 hover:border-orange-300 border cursor-pointer relative overflow-hidden">

            <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>

            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 3v2m6-2v2M9 19v2m6-2v2M5 9H3m2 6H3m18-6h-2m2 6h-2M7 19h10a2 2 0 002-2V7a2 2 0 00-2-2H7a2 2 0 00-2 2v10a2 2 0 002 2zM9 9h6v6H9V9z" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                Embodied Intelligence
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                From digital AI to physical reality
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Master Physical AI principles where robots understand physics, gravity, and real-world constraints—not just digital data.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>

          {/* Pillar 2 - Most Popular */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-6 shadow-sm hover:shadow-2xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-250 hover:border-orange-300 cursor-pointer relative overflow-hidden">
            {/* Most Popular Tag */}
            <div className="absolute top-3 pl-[4rem] -right-8 rotate-45 bg-gradient-to-r from-orange-500 to-orange-600 text-white text-[10px] font-bold px-8 py-1 shadow-lg z-20">
              MOST POPULAR
            </div>
            
            <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>

            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M13 10V3L4 14h7v7l9-11h-7z" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                ROS 2 Nervous System
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                The middleware that powers robots
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Build robotic control systems with ROS 2 nodes, topics, and services. Bridge Python AI agents to real-time robot controllers.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>

          {/* Pillar 3 */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-6 shadow-sm hover:shadow-2xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-300 hover:border-orange-300 cursor-pointer relative overflow-hidden">

            <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>

            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M21 12a9 9 0 01-9 9m9-9a9 9 0 00-9-9m9 9H3m9 9a9 9 0 01-9-9m9 9c1.657 0 3-4.03 3-9s-1.343-9-3-9m0 18c-1.657 0-3-4.03-3-9s1.343-9 3-9m-9 9a9 9 0 019-9" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                Digital Twin Simulation
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                Test safely before deploying to hardware
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Create high-fidelity simulations using Gazebo and Unity. Simulate physics, sensors, and environments before touching real robots.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>

          {/* Pillar 4 - Most Popular */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-6 shadow-sm hover:shadow-2xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-350 hover:border-orange-300 cursor-pointer relative overflow-hidden">
            {/* Most Popular Tag */}
            <div className="absolute top-3 pl-[4rem] -right-8 rotate-45 bg-gradient-to-r from-orange-500 to-orange-600 text-white text-[10px] font-bold px-8 py-1 shadow-lg z-20">
              MOST POPULAR
            </div>
            
         <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>

            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                NVIDIA Isaac Platform
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                AI-accelerated perception & navigation
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Leverage Isaac Sim for photorealistic training, Isaac ROS for SLAM, and Nav2 for autonomous navigation in complex environments.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>

          {/* Pillar 5 */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-6 shadow-sm hover:shadow-2xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-400 hover:border-orange-300 cursor-pointer relative overflow-hidden">

           <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>

            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M8 10h.01M12 10h.01M16 10h.01M9 16H5a2 2 0 01-2-2V6a2 2 0 012-2h14a2 2 0 012 2v8a2 2 0 01-2 2h-5l-5 5v-5z" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                Vision-Language-Action
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                Natural language commands to robot actions
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Integrate GPT models and Whisper for voice-driven robotics. Translate "Clean the room" into executable robot behaviors.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>

          {/* Pillar 6 */}
          <div className="group bg-[var(--color-bg)] border border-gray-200 rounded-xl p-6 shadow-sm hover:shadow-2xl transform hover:-translate-y-2 transition-all duration-300 animate-fadeIn delay-450 hover:border-orange-300 cursor-pointer relative overflow-hidden">
            
           <div className="absolute inset-0 bg-gradient-to-br from-orange-50/0 to-orange-50/0 group-hover:from-[var(--color-physical-from-pillars-card-hover-shadow)] group-hover:to-[var(--color-physical-to-pillars-card-hover-shadow)] transition-all duration-300"></div>
            <div className="relative z-10">
              <div className="flex items-center justify-center w-14 h-14 bg-orange-50 rounded-lg mb-4 group-hover:scale-110 transition-transform duration-300">
                <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M16 7a4 4 0 11-8 0 4 4 0 018 0zM12 14a7 7 0 00-7 7h14a7 7 0 00-7-7z" />
                </svg>
              </div>
              <h3 className="text-[19px] font-semibold text-orange-600 mb-2 group-hover:text-orange-700 transition-colors">
                Humanoid Development
              </h3>
              <p className="text-gray-500 text-[13px] mb-3 italic">
                Bipedal locomotion & human interaction
              </p>
              <p className="text-[var(--color-card-description-text)] text-[14px] mb-4">
                Design humanoid robots with URDF, master kinematics, balance control, and natural human-robot interaction patterns.
              </p>
              
              {/* Animated horizontal line */}
              <div className="h-[3px] rounded-full overflow-hidden">
                <div className=" h-full bg-orange-500 w-8 group-hover:w-full transition-all duration-500 ease-out"></div>
              </div>
            </div>
          </div>
        </div>

        {/* Bottom highlight */}
        {/* <div className="mt-12 bg-white border border-gray-200 rounded-xl p-6 text-center animate-fadeIn delay-500 shadow-sm hover:shadow-lg transition-all duration-300">
          <p className="text-gray-700 text-[15px] font-medium">
            <span className="text-orange-600 font-bold">13 Weeks</span> of immersive learning • 
            <span className="text-orange-600 font-bold"> Simulation-First</span> approach • 
            <span className="text-orange-600 font-bold"> Real Hardware</span> deployment
          </p>
        </div> */}
        <div className="mt-12">
          <MovingBorderDemo />
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
        .delay-250 { animation-delay: 0.25s; }
        .delay-300 { animation-delay: 0.3s; }
        .delay-350 { animation-delay: 0.35s; }
        .delay-400 { animation-delay: 0.4s; }
        .delay-450 { animation-delay: 0.45s; }
        .delay-500 { animation-delay: 0.5s; }
      `}</style>
    </section>
  );
}