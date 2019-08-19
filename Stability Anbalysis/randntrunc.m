function R = randntrunc(m,n,c)
%RANDNTRUNC Truncated normally distributed pseudorandom numbers.
%   R = RANDNTRUNC(M,N,C) returns an M-by-N matrix containing
%   pseudorandom values drawn from the standard normal
%   distribution, with samples larger (in magnitude) than C
%   discarded.

% 2015/06/17 by Benjamin Seibold

if nargin<3, c = inf; end
R = randn(m,n); % array of random samples
while 1 % loop until stopping criterion
    ind = abs(R)>c; % indices of unacceptable samples
    if ~nnz(ind), break, end % if everything acceptable, break
    R(ind) = randn(nnz(ind),1); % resample unacceptable entries
end
