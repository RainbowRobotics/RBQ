import AsyncStorage from '@react-native-async-storage/async-storage';
import type { StateStorage } from 'zustand/middleware';
import { isDesktop } from '@/lib/desktopBridge';

export const fileBackedStorage: StateStorage = {
  getItem: async (name) => {
    if (isDesktop()) {
      try {
        const r = await fetch(`/kv?key=${encodeURIComponent(name)}`);
        if (r.ok) { const v = await r.text(); if (v) return v; }
      } catch { }
    }
    return AsyncStorage.getItem(name);
  },
  setItem: async (name, value) => {
    await AsyncStorage.setItem(name, value);
    if (isDesktop()) {
      try { await fetch(`/kv?key=${encodeURIComponent(name)}`, { method: 'PUT', body: value }); } catch { }
    }
  },
  removeItem: async (name) => {
    await AsyncStorage.removeItem(name);
    if (isDesktop()) {
      try { await fetch(`/kv?key=${encodeURIComponent(name)}`, { method: 'PUT', body: '' }); } catch { }
    }
  },
};
