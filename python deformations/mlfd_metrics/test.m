% dinfo = dir('*.png');
% n=numel(dinfo);
% T=cell(n,1);
% for K = 1 : n
%   filename = dinfo(K).name;
%   filecontent = imread(filename);
%   T{K,1} = filecontent;
% end
import mlreportgen.dom.*
import mlreportgen.report.*

% To create a Word report, change the output type from "pdf" to "docx". 
% To create an HTML report, change "pdf" to "html" or "html-file" for 
% a multifile or single-file report, respectively.
rpt = Report('myreport', 'pdf');
imgStyle = {ScaleToFit(true)};
cd Area_Angle\
img1 = Image(which('AreaComparison.png'));
img1.Style = imgStyle;
cd ..
cd Curvature_Conservation_Angle\
img2 = Image(which('Curvature_ConservationComparison.png'));
img2.Style = imgStyle;
cd ..

lot = Table({img1, ' ', img2});
%lot = Table({img1, img2});
lot.entry(1,1).Style = {Width('3.2in'), Height('3in')};
lot.entry(1,2).Style = {Width('.2in'), Height('3in')};
lot.entry(1,3).Style = {Width('3.2in'), Height('3in')};

lot.Style = {ResizeToFitContents(true), Width('100%')};

add(rpt, lot);
close(rpt);
rptview(rpt);

uitable('Data',lot{:,:},'ColumnName',lot.Properties.VariableNames,...
    'RowName',lot.Properties.RowNames,'Units', 'Normalized', 'Position',[0, 0, 1, 1]);