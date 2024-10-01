import { useQuery } from "@tanstack/react-query";
import axios from "axios";
import { API_ENDPOINT } from "@/constants";


// ----------------- Get a list of containers -----------------

export const useGetContainers = () => {
  return useQuery({
    queryKey: "containers",
    queryFn: getContainers,
  });
};

export const getContainers = async () => {
  console.log("API_ENDPOINT", API_ENDPOINT);
  const response = await axios.get(`${API_ENDPOINT}/docker/ls`);
  console.log("response", response);
  return response.data;
};

// ---------- Get Logging Status for a container ----------

export const useGetLoggingStatus = () => {
  return useQuery({
    queryKey: ["loggingStatus"],
    queryFn: getLoggingStatus,
    refetchInterval: 1000,
  });
}

export const getLoggingStatus = async () => {
  const response = await axios.get(`${API_ENDPOINT}/docker/logging/status`);
  return response.data;
}

// ----------- Axios calls to start and stop logging -------------

export const startLogging = async () => {
  await axios.get(`${API_ENDPOINT}/docker/logging/start`);
}

export const stopLogging = async () => {
  await axios.get(`${API_ENDPOINT}/docker/logging/stop`);
}