import {describe,it,expect} from 'vitest';
import {eeSpeedPercent,EE_SPEED_DEFAULTS} from './armEeSpeed';
describe('EE speed preference',()=>{
 it('bounds and rounds finite values and recovers malformed persisted settings',()=>{
  expect(EE_SPEED_DEFAULTS).toEqual({move:30,rotate:20});
  for(const [value,want] of [[-1,10],[0,10],[10,10],[42.6,43],[100,100],[1000,100]])expect(eeSpeedPercent(value,30)).toBe(want);
  for(const value of [NaN,Infinity,-Infinity,undefined] as number[])expect(eeSpeedPercent(value,20)).toBe(20);
 });
});
