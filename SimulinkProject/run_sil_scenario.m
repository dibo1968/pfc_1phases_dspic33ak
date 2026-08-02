% Headless SiL run of the bug-fix verification scenario (profile 1:
% precharge from 0 V -> no load -> 96.3 ohm step at t = 0.8 s -> t = 1.2 s).
% Launched detached (timer) so MCP request timeouts cannot abort the sim;
% progress and errors go to sil_run_bugfix.log, results to sil_run_bugfix.mat.
cd('C:\Users\Andy\Documents\PFC\pfc_1phases_dspic33ak\SimulinkProject');
fid = fopen('sil_run_bugfix.log','w');
fprintf(fid, '%s starting\n', char(datetime('now'))); fclose(fid);
try
    run('mchp_pfc_foc_dsPIC33A_data.m');   % clear all + plant/gains/bus defs/profile
    % NOTE: no tic/toc across sim() - loading pfc_simulation fires its
    % PostLoadFcn, which runs the data script again and its `clear all` wipes
    % every base-workspace variable defined before the sim call. Wall time
    % comes from the log timestamps instead.
    out = sim('pfc_simulation');
    save('sil_run_bugfix.mat','out','-v7.3');
    fid = fopen('sil_run_bugfix.log','a');
    fprintf(fid, '%s done\n', char(datetime('now'))); fclose(fid);
catch e
    fid = fopen('sil_run_bugfix.log','a');
    fprintf(fid, '%s ERROR: %s\n', char(datetime('now')), ...
            getReport(e,'basic','hyperlinks','off')); fclose(fid);
end
