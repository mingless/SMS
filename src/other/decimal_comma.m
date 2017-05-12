%The MIT License (MIT)
%
%Copyright (c) 2012 Adam Heinrich <adam@adamh.cz>
%
%Permission is hereby granted, free of charge, to any person obtaining a copy
%of this software and associated documentation files (the "Software"), to deal
%in the Software without restriction, including without limitation the rights
%to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
%copies of the Software, and to permit persons to whom the Software is
%furnished to do so, subject to the following conditions:
%
%The above copyright notice and this permission notice shall be included in all
%copies or substantial portions of the Software.
%
%THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
%IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
%FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
%AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
%LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
%OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
%SOFTWARE.



function decimal_comma(axis_handle, axis_name, varargin)
%DECIMAL_COMMA - decimal comma in 2-D plot
%
%   A simple function to replace decimal points with decimal commas (which
%   are usual in Europe) in Matlab or Octave plots.
%
%   DECIMAL_COMMA(axis_handle, axis_name) changes decimal point to decimal
%   comma in a plot. Use gca for current axes handle and one of 'X', 'Y' or
%   'XY' for axis_name.
%
%   DECIMAL_COMMA(axis_handle, axis_name, formatstr) changes decimal point
%   to decimal comma in a plot. Number format is specified by formatstr
%   (see SPRINTF for details).

% (c) 2012 Adam Heinrich <adam@adamh.cz>. Published under the MIT license.

if (nargin < 2 || nargin > 3)
        error('Wrong number of input parameters.');
    end

    switch axis_name
    case 'XY'
            decimal_comma(axis_handle, 'X', varargin{:});
            decimal_comma(axis_handle, 'Y', varargin{:});

        case {'X', 'Y'}
            tick = get(axis_handle, strcat(axis_name, 'Tick'));

            n = length(tick);
            labels = cell(1,n);

            for i = 1:n
                label = num2str(tick(i), varargin{:});
                labels{i} = strrep(label, '.', ',');
            end

            %labels{1} = '';
            %labels{n} = '';

            set(axis_handle, strcat(axis_name, 'TickLabel'), labels);

        otherwise
            error('Wrong axis name! Use one of X, Y or XY.');
    end
end
