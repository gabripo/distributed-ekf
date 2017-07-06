function x_sym = symbolizer(x, tag)

x_sym = sym(zeros(1, length(x)));
for i=1:length(x)
    x_sym(i) = sym(strcat(tag, sprintf('%d', i))); 
end