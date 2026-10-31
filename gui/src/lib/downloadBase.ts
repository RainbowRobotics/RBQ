export const DOWNLOAD_BASE = (process.env.EXPO_PUBLIC_DOWNLOAD_BASE || '').replace(/\/+$/, '');
export const downloadHeaders: Record<string, string> = DOWNLOAD_BASE ? { 'X-Download-Base': DOWNLOAD_BASE } : {};
