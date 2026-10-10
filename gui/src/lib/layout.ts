import { useWindowDimensions } from 'react-native';

export const useCompactH = () => useWindowDimensions().height < 500;

export const useCompactW = () => useWindowDimensions().width < 1000;

export const useTinyW = () => useWindowDimensions().width < 720;
