function u = to_uint32(x)
%TO_UINT32  模拟 C 把 float 转成 uint32_t：先向零截断，再按 32 位回绕。
x = fix(double(x));
u = uint32(mod(x, 2^32));
end
