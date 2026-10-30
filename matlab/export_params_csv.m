function export_params_csv(p, filename)
% EXPORT_PARAMS_CSV  Write the aircraft_params() struct to a flat
% "name,value" CSV so the C++ side loads the IDENTICAL numbers instead of
% a hand-copied, independently-maintained constant table that could
% silently drift out of sync with MATLAB. This file is the single source
% of truth for the airframe; cpp/src/params.cpp only parses it.

fields_to_export = {'m','g','S','b','c','Jx','Jy','Jz','Jxz', ...
    'Gamma1','Gamma2','Gamma3','Gamma4','Gamma5','Gamma6','Gamma7','Gamma8', ...
    'rho','T_max', ...
    'CL0','CL_alpha','CL_de','CD0','CD_alpha', ...
    'Cm0','Cm_alpha','Cm_q','Cm_de', ...
    'CY_beta','CY_dr','Cl_beta','Cl_p','Cl_r','Cl_da', ...
    'Cn_beta','Cn_p','Cn_r','Cn_da','Cn_dr', ...
    'de_max','da_max','dr_max'};

fid = fopen(filename, 'w');
if fid == -1
    error('export_params_csv:openFailed', 'Could not open %s for writing.', filename);
end
fprintf(fid, '# aircraft parameters exported from MATLAB aircraft_params.m\n');
fprintf(fid, '# name,value\n');
for i = 1:numel(fields_to_export)
    name = fields_to_export{i};
    fprintf(fid, '%s,%.17g\n', name, p.(name));
end
fclose(fid);

end
