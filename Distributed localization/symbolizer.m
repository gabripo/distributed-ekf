function x_sym = symbolizer(x, tag)
%% Function to generate a symbolic array of dimension Length(x) named by tag

x_sym = sym(zeros(1, length(x)));
for i=1:length(x)
    x_sym(i) = sym(strcat(tag, sprintf('%d', i))); 
    assume(x_sym(i), 'real');
end