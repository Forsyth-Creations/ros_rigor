FROM python:3.10-slim
# Set the working directory
WORKDIR /backend

# install the opencv dependencies
# Set teh debian frontend to noninteractive
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y libglib2.0-0 libsm6 libxrender1 libxext6 ffmpeg -y

RUN pip install poetry==1.8.3

RUN apt update && apt install libpq-dev build-essential -y

# Copy the requirements file
COPY  pyproject.toml poetry.lock ./
RUN poetry config virtualenvs.create false --local && poetry install

COPY ./app ./app
COPY local.py local.py

# Expose the port
EXPOSE 5000

CMD ["python", "local.py"]