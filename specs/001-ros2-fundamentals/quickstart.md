# Quickstart Guide

This guide provides instructions for setting up the Docusaurus environment and running the book website locally.

## Prerequisites

- Node.js (v18 or later)
- npm or yarn

## Setup and Local Development

1.  **Clone the repository**:
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Install dependencies**:
    ```bash
    npm install
    # or
    yarn install
    ```

3.  **Run the development server**:
    ```bash
    npm start
    # or
    yarn start
    ```

This will start a local development server and open up a browser window. Most changes are reflected live without having to restart the server.

## Build

To generate a static build of the website, run:

```bash
npm run build
# or
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.
