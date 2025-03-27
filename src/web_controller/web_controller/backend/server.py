# Write me a runner for uvicorn

import uvicorn
import os
from app.server import app

# Get the host from the environment
host = os.getenv("HOST", "0.0.0.0")
port = int(os.getenv("PORT", 5000))


if __name__ == "__main__":
    uvicorn.run(app, host=host, port=port, log_level="error")