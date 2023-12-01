global swi1;

fig = uifigure("Name","My App");
swi1 = uiswitch(fig);
swi1.Position = [80 350 45 20];
swi1.Value = 'On';
uilabel(fig,'Position',[180 350 200 20],'Text','Stop the simulation');