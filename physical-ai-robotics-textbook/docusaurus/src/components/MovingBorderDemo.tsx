"use client";
import React from "react";
import { Button } from "../ui/moving-border";

export function MovingBorderDemo() {
  return (
    <div>
      <Button
        borderRadius="1.75rem"
        className="bg-[var(--color-bg)] dark:text-white border-orange-500 dark:border-slate-800"
      >
       <p className="text-[var(--color-text)] text-[15px] font-medium">
            <span className="text-orange-600 font-bold">13 Weeks</span> of immersive learning • 
            <span className="text-orange-600 font-bold"> Simulation-First</span> approach • 
            <span className="text-orange-600 font-bold"> Real Hardware</span> deployment
          </p>
      </Button>
    </div>
  );
}
