%%%Convert the LASA dataset from .mat format to .h5 format%%%
names = {'Angle','BendedLine','CShape','DoubleBendedLine','GShape',...
         'heee','JShape','JShape_2','Khamesh','Leaf_1',...
         'Leaf_2','Line','LShape','NShape','PShape',...
         'RShape','Saeghe','Sharpc','Sine','Snake',...
         'Spoon','Sshape','Trapezoid','Worm','WShape','Zshape',...
         'Multi_Models_1','Multi_Models_2','Multi_Models_3','Multi_Models_4'};
for i=1:length(names)
    load(['DataSet/' names{i}],'demos','dt') %loading the model
    for j=1:length(demos)
        h5create('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/pos'],[2 1000])
        data_pos = demos{j}.pos;
        h5write('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/pos'], data_pos)
        
        h5create('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/t'],[1 1000])
        data_t = demos{j}.t;
        h5write('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/t'], data_t)
        
        h5create('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/vel'],[2 1000])
        data_vel = demos{j}.vel;
        h5write('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/vel'], data_vel)
        
        h5create('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/acc'],[2 1000])
        data_acc = demos{j}.acc;
        h5write('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/acc'], data_acc)
        
        h5create('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/dt'],[1 1])
        data_dt = demos{j}.dt;
        h5write('lasa_dataset.h5',['/' names{i} '/demo' num2str(j) '/dt'], data_dt)
    end
end