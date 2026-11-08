# HARP-Bench website

Public project page: https://harp-bench.vercel.app/

## Edit and preview

- `index.html`: homepage content, result tables, styling, and video references.
- `media/`: published videos, captions, and poster images.
- `browser-control/`: interactive browser demo and its assets.

No build step is required. Run `python3 -m http.server 8000` from this directory and open http://localhost:8000.

## Collaborate

Create a branch, make changes, preview locally, then open a pull request against `main`. Keep numerical results consistent between the main and safety tables. Do not commit environment files, tokens, or local Vercel configuration.

## Deployment

The live Vercel project is `harp-bench` in `yun-zongs-projects`. GitHub automatic deployment is not connected yet. When connecting this repository to the existing project, use the repository root, Framework Preset **Other**, and no build command. Use preview deployments for pull requests and `main` for production.
