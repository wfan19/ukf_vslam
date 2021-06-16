classdef State
% State(r, v, q, tab_tags)
    
    properties
        r_body
        v_body
        q_body
        b_omega
        b_f
        tab_tags
    end
    
    methods
        % TODO: Implement cstor
        function obj = State(r, v, q, tab_tags)
            obj.r_body = r(:);
            obj.v_body = v(:);
            obj.q_body = q(:);
            obj.tab_tags = tab_tags;
            obj.b_omega = zeros(3, 1);
            obj.b_f = zeros(3, 1);
        end
        
        %% Vector <-> Matrix manifold representations
        % Functions that map between the vector and matrix representations
        % of the matrix elements
        function vec_out = to_vec(obj)
            %mat_tags = table2array(
            vec_out = [obj.r_body; obj.v_body; obj.q_body;];
        end
        
        %% Hat and Vee: See https://arxiv.org/pdf/1812.01537.pdf
        % Hat: Maps representation of Lie algebra from R^n to matrix.
        %{
        TODO: Are these going to be static functions? Or should they go
        into a separate Lie algebra class?
        function vec_out = hat( )
            
        end
        
        % Vee: Maps from matrix representation of Lie algebra to vector
        % form. "Serializes" the Lie algebra element
        function mat_out = vee()
            
        end
        %}
    end
end

