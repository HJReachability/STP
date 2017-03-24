function SPPwDisturbanceRTT_cpp(...
    problem_name, ...
    CPP_file, ...
    separatedNIRS, ...
    recomputeRTTRS, ...
    restart, ...
    low_memory,...
    project_name,...
    extraArgs)
% SPPwDisturbanceRTT()
%     Solves the entire SPP with disturbances problem using the RTT method

if nargin < 2
  CPP_file = false;
end

if nargin < 3
  separatedNIRS = false;
end

if nargin < 4
    recomputeRTTRS = false;
end

if nargin < 5
  restart = false;
end

if nargin < 6
  low_memory = false;
end

if nargin < 7
  project_name = '';
end

if nargin < 8
  extraArgs = [];
end

if CPP_file
    if separatedNIRS
        if isempty(project_name)
            if low_memory
                project_name = 'SPPProblem_17190.285834';
            else
                project_name = 'SPPProblem_17179.936194';
            end
        end
        folder = sprintf('./data/%s/%s', project_name, project_name);
        SPPP_filename = sprintf('%s_SPPP.mat', folder);
        RTTRS_filename = sprintf('%s_RTTRS.mat', folder);
        NI_RS_filename = sprintf('%s_computeNIRS.mat', folder);
%        NI_RS_DEBUG_filename = sprintf('%s_Plane_data_Plane', folder);
        NI_RS_DEBUG_filename = sprintf('%s_computeNIRS_chkpt_Plane', folder);
%        NI_RS_chkpt_filename = [];
        NI_RS_chkpt_filename = sprintf('%s_computeNIRS_chkpt_Plane', folder)
    else
        if isempty(project_name)
            if low_memory
                project_name = 'SF_dstb_11_4sSep_tMaxOrg_wLowMem';
            else
                project_name = 'SF_dstb_11_4sSep_tMaxOrg_woLowMem';
            end
            folder = sprintf('./inputs/%s', project_name);
        else
            folder = sprintf('./data/%s/%s', project_name, project_name);
        end
        SPPP_filename = sprintf('%s_SPPP.mat', folder);
        RTTRS_filename = sprintf('%s_RTTRS.mat', folder);
        NI_RS_filename = sprintf('%s_computeNIRS.mat', folder);
        NI_RS_DEBUG_filename = [];
        NI_RS_chkpt_filename = [];
    end
    if recomputeRTTRS
        CPP_RTTRS_file = false;
    else
        CPP_RTTRS_file = true;
    end
    if restart
        CPP_NIRS_file = false;
    else
        CPP_NIRS_file = true;
    end
else
        project_name = 'SPPProblem_736692.356442';
        folder = sprintf('./%s', project_name);
        SPPP_filename = sprintf('%s/SPPP.mat', folder);
        RTTRS_filename = sprintf('%s/RTTRS.mat', folder);
        NI_RS_filename = sprintf('%s/computeNIRS.mat', folder);
        NI_RS_DEBUG_filename = [];
        NI_RS_chkpt_filename = sprintf('%s/computeNIRS_chkpt.mat', folder);
        CPP_RTTRS_file = false;
        CPP_NIRS_file = false;
end

extraArgs.SPPP_filename = SPPP_filename;

if isfield(extraArgs, 'SPPP')
  SPPP = extraArgs.SPPP;
else
    if isempty(extraArgs.SPPP_filename)
        SPPP = SPPProblem(problem_name, extraArgs);
    else
        SPPP = loadSPPProblem(problem_name, extraArgs);
    end
end

% % RTT parameters for ACC and TCST papers
% vReserved = [0.25 -0.25];
% wReserved = -0.4;
% trackingRadius = 0.075;

SPPP.NI_RS_filename = NI_RS_filename;
if ~recomputeRTTRS
    SPPP.RTTRS_filename = RTTRS_filename;
end
if ~isempty(NI_RS_chkpt_filename)
    SPPP.NI_RS_chkpt_filename = NI_RS_chkpt_filename;
end

SPPP.computeRTTRS();
SPPP.computeNIRS(restart, low_memory, CPP_RTTRS_file, separatedNIRS);
if separatedNIRS && ~isempty(NI_RS_DEBUG_filename)
    SPPP.NI_RS_filename = NI_RS_DEBUG_filename;
end
SPPP.simulateNI(1, 1, SPPP.NI_RS_filename, CPP_RTTRS_file, CPP_NIRS_file, separatedNIRS);
end
