% Headless SiL runner. Select the scenario by editing SimIndex in
% SimulationProfile.m, then launch DETACHED when driving MATLAB over the MCP
% bridge (a plain evaluate gets aborted by the request timeout):
%   t = timer('StartDelay',1,'TimerFcn', ...
%       'run(''C:\Users\Andy\Documents\PFC\pfc_1phases_dspic33ak\SimulinkProject\run_sil_scenario.m'')');
%   start(t);
% Progress/errors -> sil_run_status.log, results -> sil_run_profile<N>.mat
% (both gitignored). Wall time is ~13 min per simulated second.
cd('C:\Users\Andy\Documents\PFC\pfc_1phases_dspic33ak\SimulinkProject');
fid = fopen('sil_run_status.log','w');
fprintf(fid, '%s starting\n', char(datetime('now'))); fclose(fid);
try
    run('mchp_pfc_foc_dsPIC33A_data.m');   % clear all + plant/gains/bus defs/profile
    fid = fopen('sil_run_status.log','a');
    fprintf(fid, '%s profile %d (%s), SimTime %g s\n', ...
            char(datetime('now')), SimIndex, SimName, SimTime); fclose(fid);
    % NOTE: no tic/toc (or any precious variable) across sim() - loading
    % pfc_simulation fires its PostLoadFcn, which runs the data script again
    % and its `clear all` wipes the base workspace. SimIndex/SimName survive
    % only because the re-run re-creates them with the same values. Wall time
    % comes from the log timestamps.
    out = sim('pfc_simulation');
    save(sprintf('sil_run_profile%d.mat', SimIndex), 'out', '-v7.3');
    fid = fopen('sil_run_status.log','a');
    fprintf(fid, '%s done -> sil_run_profile%d.mat (%s)\n', ...
            char(datetime('now')), SimIndex, SimName); fclose(fid);
catch e
    fid = fopen('sil_run_status.log','a');
    fprintf(fid, '%s ERROR: %s\n', char(datetime('now')), ...
            getReport(e,'basic','hyperlinks','off')); fclose(fid);
end
