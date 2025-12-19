import { ReactNode } from "react";
// import Chatbot from "../components/Chatbot";

interface RootProps {
  children: ReactNode;
}

export default function Root({ children }: RootProps) {
  return (
    <>
      {children}
      {/* <Chatbot /> */}
    </>
  );
}
