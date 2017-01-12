function s=getfeature_kp(img)
bb=getboundingbox(img);
%s=[bb(1) bb(2) bb(1)+bb(3) bb(2) bb(1) bb(2)+bb(4) bb(1)+bb(3) bb(2)+bb(4)];
%s=[bb(1) bb(2) bb(1) bb(2)+bb(4)];
s=[bb(2) bb(1) bb(2)+bb(4) bb(1)];
s=s';
end
