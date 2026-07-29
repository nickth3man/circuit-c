/*
 * serve.js — a static server with exactly one trick: two roots, tried in order.
 *
 * The inspector is source (tools/visual/) and the cards it reads are build output
 * (artifacts/corpus-cards/). Copying one into the other would mean a stale copy the first time
 * someone forgot to re-dump, so instead the server overlays them: inspector.html resolves from
 * the source tree, cards.json and every PNG from the artifacts tree, and a missing card is a
 * loud 404 rather than yesterday's picture.
 *
 * No dependencies — node's own http/fs. Runs under Playwright's webServer and standalone.
 */
const http = require("node:http");
const fs = require("node:fs");
const path = require("node:path");

const REPO = path.resolve(__dirname, "..", "..");
const ROOTS = [__dirname, path.join(REPO, "artifacts", "corpus-cards")];
const PORT = Number(process.env.PORT || 4173);

const TYPES = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".json": "application/json; charset=utf-8",
  ".png": "image/png",
  ".css": "text/css; charset=utf-8",
};

const server = http.createServer((req, res) => {
  const url = new URL(req.url, "http://localhost");
  let rel = decodeURIComponent(url.pathname);
  if (rel === "/") rel = "/inspector.html";

  // Contain the served set to the two roots: a resolved path that escapes either one is a 403,
  // so a crafted URL cannot walk out into the rest of the repo.
  for (const root of ROOTS) {
    const file = path.resolve(root, "." + rel);
    if (!file.startsWith(root + path.sep) && file !== path.join(root, path.basename(file))) continue;
    if (!file.startsWith(root)) continue;
    if (fs.existsSync(file) && fs.statSync(file).isFile()) {
      res.writeHead(200, {
        "content-type": TYPES[path.extname(file)] || "application/octet-stream",
        "cache-control": "no-store",
      });
      fs.createReadStream(file).pipe(res);
      return;
    }
  }

  res.writeHead(404, { "content-type": "text/plain" });
  res.end(
    `404 ${rel}\n\nLooked in:\n${ROOTS.join("\n")}\n\n` +
      `If cards.json is missing, run:\n  drifty_tests.exe --dump-corpus-cards artifacts/corpus-cards\n`
  );
});

server.listen(PORT, () => {
  console.log(`inspector on http://localhost:${PORT}/`);
});
