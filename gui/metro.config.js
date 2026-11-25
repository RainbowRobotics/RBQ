const { getDefaultConfig } = require('expo/metro-config');
const path = require('path');

const config = getDefaultConfig(__dirname);
config.resolver.assetExts.push('glb', 'gltf', 'obj', 'mtl', 'bin', 'stl', 'STL');
config.resolver.assetExts.push('wasm');

const prevBlockList = config.resolver.blockList;
config.resolver.blockList = [
  ...(Array.isArray(prevBlockList) ? prevBlockList : prevBlockList ? [prevBlockList] : []),
  /[\\/]public[\\/]mujoco[\\/]/,
];

const threeModule = path.resolve(__dirname, 'node_modules/three/build/three.module.js');
const prevResolveRequest = config.resolver.resolveRequest;
config.resolver.resolveRequest = (context, moduleName, platform) => {
  if (moduleName === 'three') {
    return { type: 'sourceFile', filePath: threeModule };
  }
  if ((moduleName === 'fs' || moduleName === 'path') && context.originModulePath?.includes('draco3d')) {
    return { type: 'empty' };
  }
  return prevResolveRequest
    ? prevResolveRequest(context, moduleName, platform)
    : context.resolveRequest(context, moduleName, platform);
};

if (process.env.RBQ_METRO_CACHE_URL) {
  const crypto = require('crypto');
  const { getConfig } = require('expo/config');
  const { HttpGetStore } = require('metro-cache');
  const http = require('http');
  const zlib = require('zlib');
  const url = process.env.RBQ_METRO_CACHE_URL;
  const appConfig = getConfig(__dirname, { isPublicConfig: true, skipSDKVersionRequirement: true }).exp;
  const publicEnv = Object.keys(process.env).filter((k) => k.startsWith('EXPO_PUBLIC_')).sort().map((k) => [k, process.env[k]]);
  config.cacheVersion = crypto.createHash('sha256')
    .update(JSON.stringify([config.cacheVersion, appConfig, publicEnv])).digest('hex');
  const agent = new http.Agent({ keepAlive: true, maxSockets: 16 });
  class SharedStore extends HttpGetStore {
    async set(key, value) {
      if (!process.env.RBQ_METRO_CACHE_PUSH) return;
      const raw = Buffer.isBuffer(value) ? Buffer.concat([Buffer.from([0]), value]) : Buffer.from(JSON.stringify(value) ?? 'null');
      const body = await new Promise((ok, fail) => zlib.gzip(raw, (e, b) => (e ? fail(e) : ok(b))));
      await new Promise((ok, fail) => {
        const req = http.request(`${url}/${key.toString('hex')}`, { method: 'PUT', agent, timeout: 10000 }, (res) => {
          res.resume();
          res.statusCode < 300 ? ok() : fail(new Error(`PUT ${res.statusCode}`));
        });
        req.on('timeout', () => req.destroy(new Error('PUT timed out')));
        req.on('error', fail);
        req.end(body);
      });
    }
  }
  config.cacheStores = [...config.cacheStores, new SharedStore({ endpoint: url, timeout: 3000 })];
}

module.exports = config;
