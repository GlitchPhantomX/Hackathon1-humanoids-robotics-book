import React from "react";

export default function PhysicalAICTA() {
  return (
    <section className="py-20 ">
      <div className="container mx-auto px-4">
        <div className="max-w-5xl mx-auto">
          <div className="relative group bg-gradient-to-br from-[var(--color-physical-from-CTA-background)] to-[var(--color-physical-to-CTA-background)] border-[1px] border-orange-200 rounded-2xl p-9 shadow-2xl hover:shadow-3xl transition-all duration-500 overflow-hidden animate-fadeIn">

            <div className="absolute top-0 right-0 w-64 h-64 bg-orange-200/20 rounded-full blur-3xl group-hover:scale-110 transition-transform duration-700"></div>
            <div className="absolute bottom-0 left-0 w-48 h-48 bg-orange-300/10 rounded-full blur-2xl group-hover:scale-110 transition-transform duration-700"></div>

            <div className="relative z-10">
              {/* Icon */}
              <div className="flex justify-between items-center">
                <div className="flex justify-start items-center gap-5">
                  <div className="flex justify-start animate-bounce-slow ">
                    <div className="flex items-center justify-center w-14 h-14 bg-gradient-to-br from-orange-400 to-orange-600 rounded-2xl shadow-lg group-hover:scale-110 transition-transform duration-300">
                      <svg
                        className="w-9 h-9 text-white"
                        fill="none"
                        stroke="currentColor"
                        viewBox="0 0 24 24"
                      >
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M13 10V3L4 14h7v7l9-11h-7z"
                        />
                      </svg>
                    </div>
                  </div>

                  <div className="flex justify-center items-start flex-col">
                    <h2 className="text-4xl md:text-[35px] font-bold text-center mb-4 text-[var(--color-text)] tracking-tight animate-slideUp">
                      Ready to Co-Learn with AI?
                    </h2>

                    <p className=" text-[15px] text-orange-600 font-normal mb-6 animate-fadeIn delay-200">
                      Join the revolution where robotics becomes intelligent and
                      machines <br /> become collaborative
                    </p>
                  </div>
                </div>
                <div className="flex flex-col sm:flex-row gap-4 justify-center items-center animate-fadeIn delay-500">
                  <button className="group/btn px-8 py-4 bg-gradient-to-r from-orange-500 to-orange-600 text-white font-semibold rounded-xl shadow-lg hover:shadow-2xl hover:scale-105 transition-all duration-300 flex items-center gap-2">
                    <span>Start Your Journey</span>
                    <svg
                      className="w-5 h-5 group-hover/btn:translate-x-1 transition-transform duration-300"
                      fill="none"
                      stroke="currentColor"
                      viewBox="0 0 24 24"
                    >
                      <path
                        strokeLinecap="round"
                        strokeLinejoin="round"
                        strokeWidth={2}
                        d="M13 7l5 5m0 0l-5 5m5-5H6"
                      />
                    </svg>
                  </button>
                </div>
              </div>
            </div>
          </div>

          <div className="">
            <div className=" pt-10 border-t border-orange-200 grid grid-cols-3 gap-8 text-center animate-fadeIn delay-600">
              <div>
                <div className="text-3xl font-bold text-orange-600 mb-1">
                  13
                </div>
                <div className="text-sm text-[var(--color-description-text)]">Weeks of Learning</div>
              </div>
              <div>
                <div className="text-3xl font-bold text-orange-600 mb-1">4</div>
                <div className="text-sm text-[var(--color-description-text)]">Core Modules</div>
              </div>
              <div>
                <div className="text-3xl font-bold text-orange-600 mb-1">1</div>
                <div className="text-sm text-[var(--color-description-text)]">Capstone Project</div>
              </div>
            </div>
          </div>

          <div className="mt-8 text-center animate-fadeIn delay-700">
            <p className="text-sm text-gray-500 flex items-center justify-center gap-2">
              <svg
                className="w-4 h-4 text-orange-500"
                fill="currentColor"
                viewBox="0 0 20 20"
              >
                <path
                  fillRule="evenodd"
                  d="M10 18a8 8 0 100-16 8 8 0 000 16zm3.707-9.293a1 1 0 00-1.414-1.414L9 10.586 7.707 9.293a1 1 0 00-1.414 1.414l2 2a1 1 0 001.414 0l4-4z"
                  clipRule="evenodd"
                />
              </svg>
              <span>
                No prior robotics experience required • Simulation hardware
                included • Real robot deployment optional
              </span>
            </p>
          </div>
        </div>
      </div>

      <style>{`
        @keyframes fadeIn {
          from { opacity: 0; transform: translateY(15px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes slideUp {
          from { opacity: 0; transform: translateY(30px); }
          to { opacity: 1; transform: translateY(0); }
        }
        @keyframes bounce-slow {
          0%, 100% { transform: translateY(0); }
          50% { transform: translateY(-10px); }
        }
        .animate-fadeIn { 
          animation: fadeIn 0.7s ease forwards; 
          opacity: 0;
        }
        .animate-slideUp { 
          animation: slideUp 0.8s ease forwards; 
          opacity: 0;
        }
        .animate-bounce-slow {
          animation: bounce-slow 3s ease-in-out infinite;
        }
        .delay-200 { animation-delay: 0.2s; }
        .delay-300 { animation-delay: 0.3s; }
        .delay-400 { animation-delay: 0.4s; }
        .delay-500 { animation-delay: 0.5s; }
        .delay-600 { animation-delay: 0.6s; }
        .delay-700 { animation-delay: 0.7s; }
        .shadow-3xl {
          box-shadow: 0 35px 60px -15px rgba(249, 115, 22, 0.3);
        }
      `}</style>
    </section>
  );
}
