const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('rbqDesktop', {
  shell: 'electron',
  emit: (event, payload) => ipcRenderer.send('rbq-event', event, payload),
  listen: (event, cb) => {
    const h = (_e, name, payload) => { if (name === event) cb(payload); };
    ipcRenderer.on('rbq-event', h);
    return () => ipcRenderer.removeListener('rbq-event', h);
  },
});
