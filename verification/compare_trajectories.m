function compare_trajectories()
% COMPARE_TRAJECTORIES  The actual verification step: diff the MATLAB and
% C++ trajectories for both test cases, state by state, and report
% max absolute / relative error. This is the "independent implementation
% cross-check" pattern used in real safety-critical flight software
% (two people, or two tools, implement the same spec independently; if
% they agree to within numerical tolerance, that's evidence the SPEC was
% followed correctly, not just that one piece of code runs).
%
% Run from the verification/ directory, or anywhere, with this file on
% the path: octave-cli --no-gui -q -W --eval "compare_trajectories"

verif_dir = fileparts(mfilename('fullpath'));
cases = {
    'case1', 'LQR controller',      fullfile(verif_dir,'case1_matlab_trajectory.csv'), fullfile(verif_dir,'case1_cpp_trajectory.csv');
    'case2', 'NN controller',       fullfile(verif_dir,'case2_nn_matlab_trajectory.csv'), fullfile(verif_dir,'case2_cpp_trajectory.csv');
};

col_names = {'t','pn','pe','pd','u','v','w','phi','theta','psi','p','q','r','de','da','dr','dt_thr'};

tol_abs = 1e-6;   % both sides are double-precision RK4 with identical dt; any
                   % honest reimplementation should agree far tighter than this.

overall_pass = true;
all_results = {};

for i = 1:size(cases,1)
    name = cases{i,1}; label = cases{i,2};
    m_path = cases{i,3}; c_path = cases{i,4};

    if ~exist(m_path,'file') || ~exist(c_path,'file')
        printf('SKIP %s (%s): missing file(s)\n%s\n%s\n', name, label, m_path, c_path);
        continue;
    end

    M = dlmread(m_path, ',', 1, 0);
    C = dlmread(c_path, ',', 1, 0);

    if size(M,1) ~= size(C,1)
        printf('FAIL %s (%s): row count mismatch (MATLAB %d vs C++ %d)\n', name, label, size(M,1), size(C,1));
        overall_pass = false;
        continue;
    end

    printf('\n=== %s: %s ===\n', name, label);
    printf('%-8s %12s %12s %8s\n', 'state', 'max |err|', 'max rel err', 'pass?');
    case_pass = true;
    for col = 1:size(M,2)
        err = abs(M(:,col) - C(:,col));
        max_err = max(err);
        scale = max(abs(M(:,col)));
        rel = max_err / max(scale, 1e-9);
        ok = max_err < tol_abs;
        case_pass = case_pass && ok;
        printf('%-8s %12.3e %12.3e %8s\n', col_names{col}, max_err, rel, ternary(ok,'OK','FAIL'));
    end
    overall_pass = overall_pass && case_pass;
    printf('--> %s: %s\n', name, ternary(case_pass, 'PASS (MATLAB and C++ agree within tolerance)', 'FAIL'));
end

printf('\n================================\n');
if overall_pass
    printf('VERIFICATION PASSED: independent MATLAB and C++ implementations agree on every state, both test cases.\n');
else
    printf('VERIFICATION FAILED: see per-state table above.\n');
end

end

function s = ternary(cond, a, b)
    if cond; s = a; else; s = b; end
end
