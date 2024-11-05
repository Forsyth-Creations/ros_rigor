# Stage 1: Spawn the builder image

FROM node:22.6.0-slim AS builder

# --------------- Build the Application ---------------
WORKDIR /frontend

# -------------------------------- install nvm ------------------------------------------

COPY . .

RUN npm install

# RUN npm run build

EXPOSE 3000

ENV WATCHPACK_POLLING true

CMD ["npm", "run", "dev"]