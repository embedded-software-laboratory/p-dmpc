function [cycles, edgecycles] = all_elem_cycles(G)
% ALL_ELEM_CYCLES This function find all elementary cycles for a given
% adjacent matrix of a directed graph. A circuit is elementary if no vertex
% but the first and last appears twice. 
% 
% This function is returns the same outputs of the MATLAB function
% `allcycles()`, which is only supported for MATLAB verstion higher than
% 2021a. Note that this function does not work for undirected graph.
% 
% INPUT:
%   G: object of the MATLAB class `digraph`
%
% OUTPUT:
%   cycles: n-by-1 cell array contains vertices of all elementary cycles
% 
%   edgecycles: n-by-1 cell array contains edges of each cycle
% 
% Reference: DONALD B. JOHNSON - 1975 - Finding All the Elementary cycles
% of a Directed Graph*
% 
    
    if ~isMATLABReleaseOlderThan("R2021a")
        % use build-in function
        [cycles, edgecycles] = allcycles(G);
        return
    end
    
    M = adjacency(G); % returns the sparse adjacency matrix    
    n = size(M,1);
    
    Blist = cell(n,1);
    
    blocked = false(1,n);
    
    s = 1;
    cycles = {};
    stack=[];
    
        function unblock(u)
            blocked(u) = false;
            for w=Blist{u}
                if blocked(w)
                    unblock(w)
                end
            end
            Blist{u} = [];
        end
    
        function f = circuit(v, s, C)
            f = false;
            
            stack(end+1) = v;
            blocked(v) = true;
            
            for w=find(C(v,:))
                if w == s
                    cycles{end+1,1} = stack;
                    f = true;
                elseif ~blocked(w)
                    if circuit(w, s, C)
                        f = true;
                    end
                end
            end
            
            if f
                unblock(v)
            else
                for w = find(C(v,:))
                    if ~ismember(v, Blist{w})
                        Bnode = Blist{w};
                        Blist{w} = [Bnode v];
                    end
                end
            end
            
            stack(end) = [];
        end
    
    
    while s < n
        
        % Subgraph of G induced by {s, s+1, ..., n}
        F = M;
        F(1:s-1,:) = 0;
        F(:,1:s-1) = 0;
        
        % the strongly connected components of a graph.
        [ci, sizec] = conncomp(digraph(full(F)));
    
        if any(sizec >= 2)
            
            cycle_components = find(sizec >= 2);
            least_node = find(ismember(ci, cycle_components),1);
            comp_nodes = find(ci == ci(least_node));
            
            Ak = sparse(n,n);
            Ak(comp_nodes,comp_nodes) = F(comp_nodes,comp_nodes);        
        
            s = comp_nodes(1);
            blocked(comp_nodes) = false;
            Blist(comp_nodes) = cell(length(comp_nodes),1);
            circuit(s, s, Ak);
            s = s + 1;
        
        else
            break;        
        end
    end

    n_cycles = length(cycles);

    edgecycles = cell(n_cycles,1);
    % get edges of each cycle
    store_outedges = cell(n,1);
    for i = 1:n_cycles
        cycle_i = cycles{i};
        n_nodes = length(cycle_i);
        edgecycle_i = zeros(1,n_nodes);
        for j = 1:n_nodes
            current = cycle_i(j);
            if j==n_nodes
                next = cycle_i(1); % loop back
            else
                next = cycle_i(j+1);
            end
            if isempty(store_outedges{current})
                [store_outedges{current}(:,1), store_outedges{current}(:,2)] = outedges(G,current);
            end
            eid = store_outedges{current}(:,1);
            nid = store_outedges{current}(:,2);
            
            edgecycle_i(j) = eid(nid==next);
        end
        edgecycles{i} = edgecycle_i;
    end
    


end

