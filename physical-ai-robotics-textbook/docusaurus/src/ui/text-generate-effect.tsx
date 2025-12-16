"use client";

import { useLayoutEffect } from "react";
import { motion, stagger, useAnimate } from "motion/react";
import { cn } from "../lib/utils";

interface TextGenerateEffectProps {
  words: string;
  className?: string;
  filter?: boolean;
  duration?: number;
}

export const TextGenerateEffect: React.FC<TextGenerateEffectProps> = ({
  words,
  className = "",
  filter = true,
  duration = 0.5,
}) => {
  const [scope, animate] = useAnimate<HTMLDivElement>();
  const wordsArray = words.split(" ");

  useLayoutEffect(() => {
    animate(
      "span",
      {
        opacity: 1,
        filter: filter ? "blur(0px)" : "none",
      },
      {
        duration,
        delay: stagger(0.2),
      }
    );
  }, [animate, filter, duration]);

  return (
    <div className={cn("font-semibold", className)}>
      <div className="mt-4 text-2xl leading-snug tracking-wide">
       <motion.div ref={scope}>
  {wordsArray.map((word, idx) => (
    <motion.span
      key={`${word}-${idx}`}
      className="opacity-0 text-white"
      style={{
        filter: filter ? "blur(10px)" : "none",
      }}
    >
      {word}{" "}
    </motion.span>
  ))}
</motion.div>

      </div>
    </div>
  );
};
