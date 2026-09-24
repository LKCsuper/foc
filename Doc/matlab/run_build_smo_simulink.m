function run_build_smo_simulink()
this_dir = fileparts(mfilename('fullpath'));
cd(this_dir);
addpath(this_dir);
logf = fullfile(this_dir, 'smo_slx_build.log');
fid = fopen(logf, 'w');
if fid > 0
    fprintf(fid, 'MATLAB %s start build\n', version);
    fclose(fid);
end
try
    build_smo_simulink();
    fid = fopen(logf, 'a');
    if fid > 0
        fprintf(fid, '\nBUILD_OK\n');
        fclose(fid);
    end
catch me
    fid = fopen(logf, 'a');
    if fid < 0
        fid = 1;
    end
    fprintf(fid, '\nERROR\n%s\n', getReport(me, 'extended', 'hyperlinks', 'off'));
    if fid > 2
        fclose(fid);
    end
end
end
