function ppd = ppder(pp)

    breaks = pp.breaks;
    coefs  = pp.coefs;
    pieces = pp.pieces;
    order  = pp.order;
    dim    = pp.dim;

    if order <= 1
        ppd = mkpp(breaks, zeros(pieces, 1*dim));
        return;
    end

    new_order = order - 1;
    new_coefs = zeros(pieces, new_order * dim);

    for piece = 1:pieces
        for d = 1:dim
            idxFrom = (d-1)*order + 1;
            idxTo   = d*order;
            orig = coefs(piece, idxFrom:idxTo);
            deriv = zeros(1, new_order);
            for k = 1:new_order
                power = order - k;
                deriv(k) = orig(k) * power;
            end
            new_coefs(piece, (d-1)*new_order + 1 : d*new_order) = deriv;
        end
    end

    ppd.form   = 'pp';
    ppd.breaks = breaks;
    ppd.coefs  = new_coefs;
    ppd.pieces = pieces;
    ppd.order  = new_order;
    ppd.dim    = dim;
end
