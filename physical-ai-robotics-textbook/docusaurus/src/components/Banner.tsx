export default function Banner() {
    return (
        <div
            className="fixed top-0 left-0 right-0 z-[1001] flex items-center justify-center group w-full py-1.5 font-medium text-sm text-white bg-orange-500"
        >
            <div className="flex items-center gap-2 transition duration-300 px-4 py-1 rounded-full">
                <span className="text-white">
                     Build the Future of Physical AI & Humanoid Robotics — Start Your Journey Today
                </span>

                <svg
                    className="w-4 h-4"
                    viewBox="0 0 14 14"
                    fill="none"
                    xmlns="http://www.w3.org/2000/svg"
                >
                    <path
                        d="M2.91797 7H11.0846"
                        stroke="#fff"
                        strokeWidth="1.5"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                    />
                    <path
                        d="M7 2.9165L11.0833 6.99984L7 11.0832"
                        stroke="#fff"
                        strokeWidth="1.5"
                        strokeLinecap="round"
                        strokeLinejoin="round"
                    />
                </svg>
            </div>
        </div>
    );
}