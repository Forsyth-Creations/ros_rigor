# Write me a runner for uvicorn

import uvicorn
import os
from colorama import Fore
from app.server import app
import time

# Get the host from the environment
host = os.getenv("HOST", "localhost")
port = int(os.getenv("PORT", 5000))



if __name__ == "__main__":
    uvicorn.run("app.server:app", host=host, port=port, reload=True)