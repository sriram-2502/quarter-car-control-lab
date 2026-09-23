function style_quarter_car_figure(fig)
%STYLE_QUARTER_CAR_FIGURE Keep plots legible regardless of MATLAB desktop theme.
set(fig,'Color','w');
axes_handles=findall(fig,'Type','axes');
set(axes_handles,'Color','w','XColor',[.15 .15 .15], ...
    'YColor',[.15 .15 .15],'ZColor',[.15 .15 .15], ...
    'GridColor',[.65 .65 .65]);
set(findall(fig,'Type','text'),'Color',[.1 .1 .1]);
set(findall(fig,'Type','legend'),'Color','w','TextColor',[.1 .1 .1], ...
    'EdgeColor',[.65 .65 .65]);
end
