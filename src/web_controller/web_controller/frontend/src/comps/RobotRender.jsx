import React from "react";
import { Box } from "@mui/material";

const RobotRender = ({
  position,
  radius,
  angularVelocity,
  linearVelocity,
  robotAngle,
  wheelVectors,
  spacing,
  wheelAngles,
}) => {
  const offsets = [
    [spacing, -spacing],
    [spacing, spacing],
    [-spacing, spacing],
    [-spacing, -spacing],
  ];

  function computeWheelSpeed(Vx, Vy, omega, Xi, Yi) {
    return Math.sqrt((Vx - omega * Yi) ** 2 + (Vy + omega * Xi) ** 2);
  }

  function computeWheelAngle(Vx, Vy, omega, Xi, Yi) {
    return Math.atan2(Vy + omega * Xi, Vx - omega * Yi);
  }

  return (
    <>
      {/* Robot body */}
      <Box
        sx={{
          position: "absolute",
          left: position[0] - radius,
          top: position[1] - radius,
          width: radius * 2,
          height: radius * 2,
          borderRadius: "50%",
          border: "1px solid white",
        }}
      />

      {/* Wheels */}
      {offsets.map((offset, index) => {
        const rotatedX =
          offset[0] * Math.cos(robotAngle) - offset[1] * Math.sin(robotAngle);
        const rotatedY =
          offset[0] * Math.sin(robotAngle) + offset[1] * Math.cos(robotAngle);
        const wheelX = position[0] + rotatedX - radius;
        const wheelY = position[1] + rotatedY - radius;

        return (
          <Box
            key={index}
            sx={{
              position: "absolute",
              left: wheelX,
              top: wheelY,
              width: radius * 2,
              height: radius * 2,
              // borderRadius: "50%",
              border: "1px solid white",
            }}
          />
        );
      })}

      {/* Show a central vector that corresponds to the overall linear velocity */}
      <ArrowVector
        start={position}
        end={[position[0] + linearVelocity[0], position[1] - linearVelocity[1]]}
        color="blue"
      />

      {/* Superimposed arrows */}
      {offsets.map((offset, index) => {
        const rotatedX =
          offset[0] * Math.cos(robotAngle) - offset[1] * Math.sin(robotAngle);
        const rotatedY =
          offset[0] * Math.sin(robotAngle) + offset[1] * Math.cos(robotAngle);
        const wheelCenterX = position[0] + rotatedX;
        const wheelCenterY = position[1] + rotatedY;

        // Run the wheel computation here
        const wheelSpeed = computeWheelSpeed(
          linearVelocity[0],
          -linearVelocity[1],
          angularVelocity,
          rotatedX,
          rotatedY,
        );
        const wheelAngle = computeWheelAngle(
          linearVelocity[0],
          -linearVelocity[1],
          angularVelocity,
          rotatedX,
          rotatedY,
        );

        const endpoints = [
          [wheelCenterX, wheelCenterY],
          [
            wheelCenterX + wheelSpeed * Math.cos(wheelAngle),
            wheelCenterY + wheelSpeed * Math.sin(wheelAngle),
          ],
        ];

        let wheelSpeedX = wheelSpeed * Math.cos(wheelAngle);
        let wheelSpeedY = wheelSpeed * Math.sin(wheelAngle);

        // update wheel vectors
        wheelVectors.current[index] = [
          wheelSpeedX.toFixed(2),
          -wheelSpeedY.toFixed(2),
        ];

        // Update the wheel angles
        wheelAngles.current[index] = wheelAngle;

        return (
          <ArrowVector
            key={index}
            start={endpoints[0]}
            end={endpoints[1]}
            color="green"
          />
        );
      })}
    </>
  );
};

const ArrowVector = ({
  start,
  end,
  color = "red",
  strokeWidth = 2,
  label = null,
}) => {
  const arrowSize = 10; // Size of arrowhead

  // Calculate the angle of the arrow
  const angle = Math.atan2(end[1] - start[1], end[0] - start[0]);

  // Calculate arrowhead points to ensure the tip is exactly at 'end'
  const leftX = end[0] - arrowSize * Math.cos(angle - Math.PI / 6);
  const leftY = end[1] - arrowSize * Math.sin(angle - Math.PI / 6);
  const rightX = end[0] - arrowSize * Math.cos(angle + Math.PI / 6);
  const rightY = end[1] - arrowSize * Math.sin(angle + Math.PI / 6);

  if (start[0] === end[0] && start[1] === end[1]) {
    return;
  }

  return (
    <svg
      width="100%"
      height="100%"
      style={{
        position: "absolute",
        top: 0,
        left: 0,
        pointerEvents: "none",
        zIndex: 50,
      }}
    >
      {/* Main arrow line (stopping at the base of the arrowhead) */}
      <line
        x1={start[0]}
        y1={start[1]}
        x2={(leftX + rightX) / 2}
        y2={(leftY + rightY) / 2}
        stroke={color}
        strokeWidth={strokeWidth}
      />

      {/* Arrowhead with the tip at 'end' */}
      <polygon
        points={`${end[0]},${end[1]} ${leftX},${leftY} ${rightX},${rightY}`}
        fill={color}
      />

      {/* Label at the center of the vector */}
      {label && (
        <text
          x={(start[0] + end[0]) / 2}
          y={(start[1] + end[1]) / 2}
          fill={color}
          fontSize="12"
          textAnchor="middle"
          dy="-5"
        >
          {label}
        </text>
      )}
    </svg>
  );
};

export default RobotRender;
