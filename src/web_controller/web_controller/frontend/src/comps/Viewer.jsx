import {
  Box,
  Stack,
  Typography,
  Paper,
  LinearProgress,
  Tooltip,
} from "@mui/material";
import axios from "axios";
import { useQuery } from "@tanstack/react-query";

// Create a hook to pull in the wheel data
const getWheelData = async () => {
  const response = await axios.get("http://localhost:5000/swerve_data");
  return response.data;
};

const RadiansToDegrees = (radians) => {
  return - radians * (180 / Math.PI);
};

const useGetWheelData = () => {
  return useQuery({
    queryKey: ["wheelData"],
    queryFn: getWheelData,
    refetchInterval: 100, // Poll every 1 second
  });
};

function LinearProgressWithLabel(props) {
  return (
    <Box sx={{ display: "flex", alignItems: "center" }}>
      <Box sx={{ width: "100%", mr: 1 }}>
        <LinearProgress variant="determinate" {...props} />
      </Box>
      <Box sx={{ minWidth: 20 }}>
        <Typography variant="body2" sx={{ color: "text.secondary" }}>
          {props.value.toFixed(2)}
        </Typography>
      </Box>
    </Box>
  );
}

export default function Viewer({
  width = 200,
  height = 200,
}) {
  const { data } = useGetWheelData();

  let partialHeight = (height * 5) / 16;
  let partialWidth = width / 4;
  let commonProps = { height: partialHeight, width: partialWidth };

  return (
    <Paper sx={{ p: 3 }} variant="outlined">
      <Box sx={{ width: "100%" }}>
        <Typography variant="h5" align="center">
          Camera
        </Typography>
      </Box>
      {data && (
        <Box sx={{ position: "relative", width: width, height: height + 120 }}>
          <SingleWheel
            actualAngle={RadiansToDegrees(data.swerve_a.pivot_position)}
            commandedAngle={RadiansToDegrees(
              data.swerve_a?.command?.angular?.z,
            )}
            actualSpeed={data.swerve_a.speed}
            commandedSpeed={data.swerve_a?.command?.linear?.x}
            sx={{ top: 0, right: 0 }}
            name="A"
            jointName="1"
            {...commonProps}
          />
          <SingleWheel
            actualAngle={RadiansToDegrees(data.swerve_b.pivot_position)}
            commandedAngle={RadiansToDegrees(
              data.swerve_b?.command?.angular?.z,
            )}
            actualSpeed={data.swerve_b.speed}
            commandedSpeed={data.swerve_b?.command?.linear?.x}
            sx={{ bottom: 0, right: 0 }}
            name="B"
            jointName="2"
            {...commonProps}
          />
          <SingleWheel
            actualAngle={RadiansToDegrees(data.swerve_c.pivot_position)}
            commandedAngle={RadiansToDegrees(
              data.swerve_c?.command?.angular?.z,
            )}
            actualSpeed={data.swerve_c.speed}
            commandedSpeed={data.swerve_c?.command?.linear?.x}
            sx={{ bottom: 0, left: 0 }}
            name="C"
            jointName="3"
            {...commonProps}
          />
          <SingleWheel
            actualAngle={RadiansToDegrees(data.swerve_d.pivot_position)}
            commandedAngle={RadiansToDegrees(
              data.swerve_d?.command?.angular?.z,
            )}
            actualSpeed={data.swerve_d.speed}
            commandedSpeed={data.swerve_d?.command?.linear?.x}
            sx={{ top: 0, left: 0 }}
            name="D"
            jointName="4"
            {...commonProps}
          />
        </Box>
      )}
    </Paper>
  );
}

function SingleWheel({
  actualAngle = 0,
  commandedAngle = 0,
  actualSpeed = 0,
  commandedSpeed = 0,
  height,
  width,
  sx,
  name,
  jointName = "Unknown",
}) {
  const commonStyle = {
    height: width,
    width: height,
    bgcolor: "transparent",
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    border: 1,
  };

  // Check if the actualAngle and commandedAngle are within error range
  let error = Math.abs(actualAngle - commandedAngle) > 2;

  let borderThickness = 3;

  return (
    <Box
      id={`wheel-${name}`}
      sx={{
        ...sx,
        position: "absolute",
        p: "20px",
        display: "flex",
        flexDirection: "column",
        alignItems: "center",
        gap: 1, // Adds spacing between wheel and gauges
      }}
    >
      <Tooltip title={`Joint: ${jointName}`}>
        <Box sx={{ position: "relative", ...commonStyle, border: null }}>
          <Paper
            elevation={0}
            variant="outlined"
            sx={{
              ...commonStyle,
              position: "absolute",
              transform: `rotate(${actualAngle}deg)`,
              borderColor: error ? "blue" : "green",
              borderWidth: borderThickness,
            }}
            id={`wheel-${name}-actualAngle`}
          >
            <Typography align="center">{name}</Typography>
          </Paper>
          {error && (
            <Paper
              elevation={0}
              variant="outlined"
              sx={{
                ...commonStyle,
                position: "absolute",
                transform: `rotate(${commandedAngle}deg)`,
                borderColor: "red",
                borderWidth: borderThickness,
              }}
              id={`wheel-${name}-commandedAngle`}
            />
          )}
        </Box>
      </Tooltip>
      <Stack
        sx={{ width: "50px", }}
        alignItems={"center"}
        justifyContent={"center"}
      >
        <Tooltip title="Actual Speed">
          <LinearProgressWithLabel variant="determinate" value={actualSpeed} />
        </Tooltip>
        <Tooltip title="Commanded Speed">
        <LinearProgressWithLabel variant="determinate" value={commandedSpeed} />
        </Tooltip>  
        <Tooltip title="Commanded Angle">
        <Typography variant="body2">{(commandedAngle).toFixed(0) }</Typography>
        </Tooltip>
      </Stack>
    </Box>
  );
}
