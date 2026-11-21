function b(u,r,c){const f=Math.floor(u*64)/64,s=Math.fround((f-c*(r-1))/r),t=[];let e=0;for(let o=1;o<=r;o++){const n=Math.floor(s*o*64);t.push((n-e)/64),e=n}return t}export{b as gridTracks};
