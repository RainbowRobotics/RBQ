import type { ComponentType } from 'react';
import type { IconName } from '@/components/Icon';

export type WidgetCase = {
  name: string;
  from: string;
  when: string;
  Demo?: ComponentType;
  unavailable?: string;
  code?: string;
  size?: 'normal' | 'wide' | 'full';
};

export type Category = {
  key: string;
  label: string;
  icon: IconName;
  load: () => Promise<WidgetCase[]>;
};
