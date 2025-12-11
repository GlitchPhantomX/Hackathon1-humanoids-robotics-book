import React, { useState } from "react";
import { Highlight } from "./HeroHighlightDemo";
import { TextGenerateEffectDemo } from "./TextGenerateEffectDemo";
import { MovingBorderDemo } from "./MovingBorderDemo3";

const HeroSection = () => {
  const [isMobileMenuOpen, setIsMobileMenuOpen] = useState(false);

  const openMenu = () => setIsMobileMenuOpen(true);
  const closeMenu = () => setIsMobileMenuOpen(false);

  return (
    <div className="min-h-screen ">
      <style>
        {`
          @import url('https://fonts.googleapis.com/css2?family=Poppins:wght@300;400;500;600;700;800&display=swap');
          * {
            font-family: 'Poppins', sans-serif;
          }
        `}
      </style>
      <div className="relative flex flex-col items-center justify-center text-sm px-4 md:px-16 lg:px-24 xl:px-32 light:bg-white dark:bg-black text-black">
        <div className="group flex items-center gap-2 rounded-full p-1 pr-3 mt-20 bg-[var(--color-badge-background)] ">
          <span className="bg-orange-600 text-white text-xs px-3.5 py-1 rounded-full">
            NEW
          </span>

          <p className="flex items-center gap-1">
            <span className="text-[var(--color-text)]">
              Start Building Intelligent Systems
            </span>

            <svg
              xmlns="http://www.w3.org/2000/svg"
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              stroke="#FF6D1F"
              strokeWidth="2"
              strokeLinecap="round"
              strokeLinejoin="round"
              className="group-hover:translate-x-0.5 transition duration-300"
            >
              <path d="m9 18 6-6-6-6" />
            </svg>
          </p>
        </div>

        <h1 className="text-5xl leading-[80px] md:text-[50px] font-bold max-w-[56rem] text-center text-[var(--color-text)]">
          Build Intelligent Systems with Physical AI &{" "}
          <span className="bg-gradient-to-r from-orange-600 to-orange-400 px-3 text-white rounded-xl">
            Humanoid Robotics
          </span>
        </h1>

        <p className="text-base text-center text-[var(--color-description-text)] dark:text-gray-300 max-w-3xl mt-6">
          Learn how AI systems interact with the physical world. Build,
          simulate, and control humanoid robots using ROS 2, Gazebo, and NVIDIA
          Isaac—bringing embodied intelligence to life.
        </p>

      <a href="/docs/Introduction/introduction" className="hover:border">
          <div className="flex items-center gap-4 mt-8">

  <button
    className="flex items-center gap-2 bg-[var(--color-dark-button-background)] text-white 
               font-medium px-5 py-3 rounded-lg hover:bg-[var(--color-hover-button-background)] transition-all duration-300 
                hover:shadow-lg hover:-translate-y-0.5"
  >
    <svg
      xmlns="http://www.w3.org/2000/svg"
      fill="none"
      viewBox="0 0 24 24"
      strokeWidth={2}
      stroke="white"
      className="w-5 h-5"
    >
      <path
        strokeLinecap="round"
        strokeLinejoin="round"
        d="M13.5 4.5L20 12m0 0l-6.5 7.5M20 12H4"
      />
    </svg>
    Start Reading
  </button>

</div>
      </a>


        <div className="flex flex-wrap justify-center items-center gap-4 md:gap-14 mt-12">
          {[
            "Hands-on Humanoid Robotics",
            "Build AI Systems for the Physical World",
            "Learn ROS 2 & Gazebo Simulation",
          ].map((txt) => (
            <p key={txt} className="flex items-center gap-2">
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
                className="size-5 text-orange-600"
              >
                <path d="M20 6 9 17l-5-5" />
              </svg>
              <span className="text-slate-400">{txt}</span>
            </p>
          ))}
        </div>

        <div className="relative w-full max-w-6xl mt-20">
          <div className="absolute inset-0 rounded-3xl border-[1px] border-orange-500"></div>

          <div className="relative m-6 rounded-2xl overflow-hidden shadow-2xl">
            <div
              className="w-full h-64 p-8"
              style={{
                background:
                  "linear-gradient(135deg, #ff5a1f 0%, #ff7a33 25%, #ff9f3f 50%, #ffca80 75%, #ffe6c2 100%)",
              }}
            >
              <TextGenerateEffectDemo />
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default HeroSection;
