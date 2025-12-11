"use client";
import React from "react";
import { Button } from "../ui/moving-border";

export function MovingBorderDemo() {
  return (
    <div>
      <a href="/docs/Introduction/introduction">
        <Button
        borderRadius="1.75rem"
        className="bg-orange-500 text-white border-white dark:border-slate-800"
      >
       <p className="text-white text-[15px] font-medium px-[40px]">
          Start Reading
          </p>
      </Button>
      </a>
    </div>
  );
}
