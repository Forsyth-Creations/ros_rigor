import { Box, Card, Stack, Typography, Paper, LinearProgress, Tooltip } from "@mui/material";

function LinearProgressWithLabel(props) {
    return (
      <Box sx={{ display: 'flex', alignItems: 'center' }}>
        <Box sx={{ width: '100%', mr: 1 }}>
          <LinearProgress variant="determinate" {...props} />
        </Box>
        <Box sx={{ minWidth: 20 }}>
          <Typography variant="body2" sx={{ color: 'text.secondary' }}>
            {`${Math.round(props.value)}`}
          </Typography>
        </Box>
      </Box>
    );
  }

export default function Viewer({
  width = 200,
  height = 200,
  wheel_orientations,
  ...props
}) {
  let partialHeight = (height * 5) / 16;
  let partialWidth = width / 4;

  let commonProps = { height: partialHeight, width: partialWidth };

  return (
    <Paper sx={{ p: 3 }} variant="outlined">
      <Box sx={{ position: "relative", width: width, height: height + 120 }}>
        <SingleWheel
          actualAngle={wheel_orientations?.w1?.actualAngle}
          commandedAngle={wheel_orientations?.w1?.commandedAngle}
          actualSpeed={wheel_orientations?.w1?.actualSpeed}
          commandedSpeed={wheel_orientations?.w1?.commandedSpeed}
          sx={{ top: 0, left: 0 }}
          name="W1"
          {...commonProps}
        />
        <SingleWheel
          actualAngle={wheel_orientations?.w2?.actualAngle}
          commandedAngle={wheel_orientations?.w2?.commandedAngle}
          actualSpeed={wheel_orientations?.w2?.actualSpeed}
          commandedSpeed={wheel_orientations?.w2?.commandedSpeed}
          sx={{ top: 0, right: 0 }}
          name="W2"
          {...commonProps}
        />
        <SingleWheel
          actualAngle={wheel_orientations?.w3?.actualAngle}
          commandedAngle={wheel_orientations?.w3?.commandedAngle}
          actualSpeed={wheel_orientations?.w3?.actualSpeed}
          commandedSpeed={wheel_orientations?.w3?.commandedSpeed}
          sx={{ bottom: 0, left: 0 }}
          name="W3"
          {...commonProps}
        />
        <SingleWheel
          actualAngle={wheel_orientations?.w4?.actualAngle}
          commandedAngle={30}
          actualSpeed={wheel_orientations?.w4?.actualSpeed}
          commandedSpeed={30}
          sx={{ bottom: 0, right: 0 }}
          name="W4"
          {...commonProps}
        />
      </Box>
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
  ...props
}) {
  const commonStyle = {
    width: width,
    height: height,
    bgcolor: "transparent",
    display: "flex",
    justifyContent: "center",
    alignItems: "center",
    border: 1,
  };

  // Check if the actualAngle and commandedAngle are within error range
  let error = Math.abs(actualAngle - commandedAngle) > 2;
  console.log(`${name} ${actualAngle} ${commandedAngle} ${error}`);

  let borderThickness = 3;

  return (
    <Paper
      variant="outlined"
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
      <Box sx={{ position: "relative", ...commonStyle }}>
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
          <Typography  align="center">
            {name}
          </Typography>
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

      <Stack
        sx={{ display: "flex", width: "50px", justifyContent: "space-around" }}
      >
        <Tooltip title="Actual Speed">
                <LinearProgressWithLabel variant="determinate" value={actualSpeed} />
        </Tooltip>
                <LinearProgressWithLabel variant="determinate" value={commandedSpeed} />
      </Stack>
    </Paper>
  );
}
