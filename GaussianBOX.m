function [evaled_thetas,avgs,CIs] = GaussianBOX( train, test, n_samples, C, infeasible_rwd )
    %%% Inputs to the algorithm: %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % train - experience matrix; we have values for these (top rows)
    % test  - test vector; we do not have values for these (bottom row)
    % n_sampels - number of tests = k in our paper
    % C1,C2 - parameters of upper bounds
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    n_thetas = size(test,2);

    % approximate the joint mean of each theta
    data_mu = mean(train);
    data_cov = cov(train);
    
    % choose the first theta based on the best 95% confidence interval
    evaled_thetas = [];
    ts = tinv([C],size(train,1)-1);

    CI = data_mu' + ts*(1/sqrt(size(train,1)))*sqrt(diag((data_cov)));
    [~,t_idx] = max(CI);
    evaled_thetas(1) = t_idx;
    evaled_theta=t_idx;

    if exist('infeasible_rwd','var') && ( test(evaled_thetas(1))~= infeasible_rwd )
        evaled_thetas = evaled_thetas(1);
        return
    end

    remain_cols_idxs = 1:size(train,2); 
    remain_cols_idxs(remain_cols_idxs==evaled_thetas)=[];

    selected_cols = train(:,evaled_thetas);
    remain_cols = train(:,remain_cols_idxs);
    train_mean_vals = mean(train);

    n_thetas = size(train,2);
    avgs = zeros(n_samples,n_thetas);
    CIs=0;
    avgs(1,:) = data_mu;

    % compute the (joint) conditional values
    % We decompose the covariance matrix as:
    % let d = evaluated cols
    %     D = to be predicted
    % Then,
    % Cov = [cov(D,D), cov(d,D)^T);
    %        cov(D,d), cov(d,d)]
    % where cov(D,D) is the covariance of the arms to be predicted
    %       cov(d,D) is the covariance between evaluated and predict arms
    %       cov(d,d) is the covariance of the evaluated
    
    % this should be cov(remain_cols), but let's do this for now to not
    % to fiddle with the indexing
    for k=2:n_samples
        dd = data_cov(evaled_theta); dd = dd+eye(size(dd,1),size(dd,2));
       
        % size of dD = n_remain_cols by n_selected_cols = (n_to_predict,n_data)
        DD = data_cov;  % the whole matrix
        dD = DD; dD = dD(evaled_theta,:)'; % take the cov between selected column and all others
        data_mu = data_mu + (dD * inv(dd) * ( test(evaled_theta) - data_mu(evaled_theta))')';
        data_cov = DD - dD*inv(dd)*dD';
        
        % compute the 95 confidence interval of the evaled thetas
        CI = data_mu' + ts*(1/sqrt(size(train,1)))*sqrt(diag((data_cov)));
        CI(evaled_thetas) = test(evaled_thetas); % set the evaled ones to their actual values
        avgs(k,:) = data_mu;

        % choose the next best one if we ever choose the old one again
        % this problem would go away if I fiddle with the indexing...
        [~,t_idx_list] = sort(CI,'descend');
        t_idx = t_idx_list(1); idx=1;
        while any(evaled_thetas==t_idx) 
            idx = idx+1;
            t_idx = t_idx_list(idx);
        end
        
        evaled_thetas(k) = t_idx;
        if exist('infeasible_rwd','var') && ( test(evaled_thetas(k))~= infeasible_rwd )
            evaled_thetas = evaled_thetas(1);
            return
        end
        remain_cols_idxs(remain_cols_idxs==evaled_thetas(k))=[];
        selected_cols = train(:,evaled_thetas);
        remain_cols = train(:,remain_cols_idxs);
        evaled_theta = t_idx;
    end
end

