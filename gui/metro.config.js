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

module.exports = config;
