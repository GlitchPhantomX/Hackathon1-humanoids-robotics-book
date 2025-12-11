

const ThreePoints = () => {
  return (
       <div className="">
             <div className="grid grid-cols-1 md:grid-cols-3 mb-10 lg:gap-0 gap-4 animate-fadeIn delay-400 px-16 mt-5">
                <div className="flex flex-col items-center text-center group/item">

                  <div className="flex items-center justify-center w-14 h-14 bg-orange-100 rounded-xl mb-3 group-hover/item:scale-110 group-hover/item:bg-orange-200 transition-all duration-300">

                    <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.663 17h4.673M12 3v1m6.364 1.636l-.707.707M21 12h-1M4 12H3m3.343-5.657l-.707-.707m2.828 9.9a5 5 0 117.072 0l-.548.547A3.374 3.374 0 0014 18.469V19a2 2 0 11-4 0v-.531c0-.895-.356-1.754-.988-2.386l-.548-.547z" />
                    </svg>
                  </div>
                  <h4 className="font-semibold text-[var(--color-text)] mb-1">Simulation-First</h4>
                  <p className="text-sm text-[var(--color-description-text)]">Master in virtual worlds before hardware</p>
                </div>

                <div className="flex flex-col items-center text-center group/item">
                  <div className="flex items-center justify-center w-14 h-14 bg-orange-100 rounded-xl mb-3 group-hover/item:scale-110 group-hover/item:bg-orange-200 transition-all duration-300">

                    <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 6V4m0 2a2 2 0 100 4m0-4a2 2 0 110 4m-6 8a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4m6 6v10m6-2a2 2 0 100-4m0 4a2 2 0 110-4m0 4v2m0-6V4" />
                    </svg>
                  </div>
                  <h4 className="font-semibold text-[var(--color-text)] mb-1">Production-Ready</h4>
                  <p className="text-sm text-[var(--color-description-text)]">Build real autonomous systems from day one</p>
                </div>

                <div className="flex flex-col items-center text-center group/item">
                  <div className="flex items-center justify-center w-14 h-14 bg-orange-100 rounded-xl mb-3 group-hover/item:scale-110 group-hover/item:bg-orange-200 transition-all duration-300">
                    <svg className="w-7 h-7 text-orange-600" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M17 20h5v-2a3 3 0 00-5.356-1.857M17 20H7m10 0v-2c0-.656-.126-1.283-.356-1.857M7 20H2v-2a3 3 0 015.356-1.857M7 20v-2c0-.656.126-1.283.356-1.857m0 0a5.002 5.002 0 019.288 0M15 7a3 3 0 11-6 0 3 3 0 016 0zm6 3a2 2 0 11-4 0 2 2 0 014 0zM7 10a2 2 0 11-4 0 2 2 0 014 0z" />
                    </svg>
                  </div>
                  <h4 className="font-semibold text-[var(--color-text)] mb-1">Human-Centered</h4>
                  <p className="text-sm text-[var(--color-description-text)]">Design robots that understand us</p>
                </div>
              </div>

          </div>

  )
}

export default ThreePoints