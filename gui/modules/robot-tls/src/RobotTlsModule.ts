import { NativeModule, requireNativeModule } from 'expo';

export type NativeProbe = { kind: 'tls'; fp: string; serial: string | null } | { kind: 'plain' } | { kind: 'down' };

declare class RobotTlsModule extends NativeModule {
  probe(host: string, port: number): Promise<NativeProbe>;
  relay(host: string, port: number, fp: string): Promise<number>;
  close(host: string, port: number): void;
}

export default requireNativeModule<RobotTlsModule>('RobotTls');
