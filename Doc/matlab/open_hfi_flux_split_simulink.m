function open_hfi_flux_split_simulink()
%OPEN_HFI_FLUX_SPLIT_SIMULINK  重建并打开VESC式教学拆分版HFI/磁链/弱磁模型。
this_dir=fileparts(mfilename('fullpath')); addpath(this_dir);
load_system('simulink');
build_hfi_flux_split_simulink();
open_system(fullfile(this_dir,'hfi_flux_split_pmsm.slx'));
end
