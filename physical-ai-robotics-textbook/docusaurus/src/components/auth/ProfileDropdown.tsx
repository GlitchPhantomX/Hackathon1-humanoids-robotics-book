import React, { useEffect, useRef, memo } from "react";
import { User } from "./types";

interface ProfileDropdownProps {
  user: User;
  isOpen: boolean;
  onClose: () => void;
  onLogout: () => void;
}

const ProfileDropdownComponent: React.FC<ProfileDropdownProps> = ({
  user,
  isOpen,
  onClose,
  onLogout,
}) => {
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Escape Key Close
  useEffect(() => {
    if (!isOpen) return;
    const handleKey = (e: KeyboardEvent) => e.key === "Escape" && onClose();
    document.addEventListener("keydown", handleKey);
    return () => document.removeEventListener("keydown", handleKey);
  }, [isOpen, onClose]);

  // Outside Click Close
  useEffect(() => {
    if (!isOpen) return;
    const handleClick = (e: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(e.target as Node)) {
        onClose();
      }
    };
    document.addEventListener("mousedown", handleClick);
    return () => document.removeEventListener("mousedown", handleClick);
  }, [isOpen, onClose]);

  if (!isOpen) return null;

  // Format Languages
  const formatProgrammingLanguages = (languages: string | undefined) => {
    if (!languages) return "Not specified";
    try {
      const parsed = JSON.parse(languages);
      return Array.isArray(parsed) ? parsed.join(", ") : languages;
    } catch {
      return languages;
    }
  };

  return (
    <div
      ref={dropdownRef}
      className="absolute right-4 mt-2 w-80 rounded-xl shadow-xl backdrop-blur-xl 
                 bg-white/80 dark:bg-black/60 border border-white/20 dark:border-white/10 
                 animate-fadeIn p-4 z-50"
    >
      {/* Header */}
      <div className="border-b border-gray-200 dark:border-gray-700 pb-3 mb-3">
        <h3 className="text-lg font-semibold text-gray-900 dark:text-white">
          {user.name}
        </h3>
        <p className="text-sm text-gray-600 dark:text-gray-300">{user.email}</p>
      </div>

      {/* Profile Fields */}
      <div className="space-y-3">
        <Field label="Software Background" value={user.softwareBackground} />
        <Field label="Hardware Background" value={user.hardwareBackground} />
        <Field
          label="Programming Languages"
          value={formatProgrammingLanguages(user.programmingLanguages)}
        />
        <Field label="Robotics Experience" value={user.roboticsExperience} />
        <Field label="AI/ML Experience" value={user.aiMlExperience} />

        {user.hasRosExperience && <Field label="ROS Experience" value="Yes" />}
        {user.hasGpuAccess && <Field label="GPU Access" value="Yes" />}

        {user.learningGoals && (
          <Field
            label="Learning Goals"
            value={
              user.learningGoals.length > 100
                ? user.learningGoals.slice(0, 100) + "..."
                : user.learningGoals
            }
          />
        )}
      </div>

      {/* Logout */}
      <button
        onClick={onLogout}
        className="w-full mt-4 py-2 bg-red-500 hover:bg-red-600 
                   text-white font-semibold rounded-lg transition"
      >
        Logout
      </button>
    </div>
  );
};

// Reusable Field Component
const Field = ({
  label,
  value,
}: {
  label: string;
  value: string | undefined;
}) => (
  <div className="text-sm">
    <span className="font-medium text-gray-800 dark:text-gray-200">{label}:</span>{" "}
    <span className="text-gray-600 dark:text-gray-300">
      {value || "Not specified"}
    </span>
  </div>
);

const ProfileDropdown = memo(ProfileDropdownComponent);
export default ProfileDropdown;
