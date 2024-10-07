import React, { useState } from "react";
import {
  IconButton,
  Chip,
  Paper,
  Stack,
  Typography,
  Alert,
} from "@mui/material";
import { Visibility, VisibilityOff } from "@mui/icons-material";
import { LineChart } from "@mui/x-charts/LineChart";
import { useStreamLogs } from "@/hooks/DockerHooks";

export default function DockerContainer({ entry, value }) {
  const [isWatching, setIsWatching] = useState(false);
  const [parsedData, setParsedData] = useState([]);
  const { data, isLoading, isError } = useStreamLogs(value, isWatching);

  const toggleWatch = () => {
    setIsWatching((prev) => !prev);
  };

  // Parse the streamed data into chart-friendly format
  //   const parsedData = data && Array.isArray(data)
  //   ? data.flat().map((item) => ({
  //       time: new Date().toLocaleTimeString(), // Optionally, replace with an actual timestamp from item if available
  //       cpu: item.cpu_percent,
  //       mem: item.mem_percent,
  //     }))
  //   : [];

  React.useEffect(() => {
    console.log(data)
    if (data && Array.isArray(data)) {
      setParsedData(
        data.flat().map((item) => ({
          time: item.sample_number,
          cpu: item.cpu_percent,
          mem: item.mem_percent,
        })),
      );
    }
  }, [data]);

  return (
    <Paper key={entry} sx={{ p: "5px" }}>
      <Stack spacing={2}>
        <Typography>{value}</Typography>
        <Chip label={<Typography variant="caption">{entry}</Typography>} />
        <IconButton onClick={toggleWatch}>
          {isWatching ? <VisibilityOff /> : <Visibility />}
        </IconButton>
        {isLoading && <Typography>Loading...</Typography>}
        {isError && <Alert severity="error">Error fetching data</Alert>}
        {parsedData.length > 0 && (
          <LineChart
            width={600}
            height={300}
            xAxis={[
              {
                dataKey: "time",
                label: "Time",
              },
            ]}
            series={[
              {
                dataKey: "cpu",
                label: "CPU %",
                color: "#8884d8",
                showMark: false
              },
              {
                dataKey: "mem",
                label: "Memory %",
                color: "#82ca9d",
                showMark: false
              },
            ]}
            dataset={parsedData}
          />
        )}
      </Stack>
    </Paper>
  );
}
