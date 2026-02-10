function hf_strong_energy = smoothness_frequency_metrics(v, fs, fc_ratio, Amp_th, doPlot)
% v: 信号（速度或加速度等）
% fs: 采样频率 (Hz)
% fc_ratio: 高频阈值占 Nyquist 的比例（如 0.25 表示 0.25 * (fs/2)）
% Amp_th: 高频幅值阈值（只有高频且幅值 > Amp_th 的频率才算高抖动）
% doPlot: 是否绘制频谱图 (true/false)
%
% 输出（结构体 freqMetrics）：
%   total_energy        - 频域总能量（≈ 时域 sum((v-mean(v)).^2)）
%   hf_energy           - 高频能量
%   lf_energy           - 低频能量
%   hf_energy_n         - 高频能量 / 点数（可看作单位长度高频能量）
%   hf_strong_energy    - 高频且幅值>Amp_th 的能量
%   hf_strong_ratio     - hf_strong_energy / total_energy
%   hf_strong_count     - 高频且幅值>Amp_th 的频率点个数
%   f, P, Amp           - 频率轴、单边能量谱、单边幅值谱

% ---------- 默认参数 ----------
if nargin < 3 || isempty(fc_ratio)
    fc_ratio = 0.25;      % 默认 25% Nyquist
end
if nargin < 4 || isempty(Amp_th)
    Amp_th = 0;           % 默认无阈值（即不过滤高抖动，只计算总高频）
end
if nargin < 5 || isempty(doPlot)
    doPlot = false;       % 默认不画图（避免你已有代码出大量窗口）
end

% 统一列向量
v = v(:);

% 去直流
v0 = v - mean(v);

% 如果信号几乎全常数，直接返回 0 指标
if sum(v0.^2) < eps
    freqMetrics.total_energy        = 0;
    freqMetrics.hf_energy           = 0;
    freqMetrics.lf_energy           = 0;
    freqMetrics.hf_energy_n         = 0;
    freqMetrics.hf_strong_energy    = 0;
    freqMetrics.hf_strong_ratio     = 0;
    freqMetrics.hf_strong_count     = 0;
    freqMetrics.f = [];
    freqMetrics.P = [];
    freqMetrics.Amp = [];
    
    if doPlot
        figure; title('信号能量几乎为常数，频谱为空'); 
    end
    return;
end

% ---------- FFT 与单边能量谱 ----------
N = length(v0);
V = fft(v0);

% Parseval：sum(v0.^2) = (1/N) * sum(|V|.^2)
P2 = abs(V).^2 / N;        % 双边能量谱

% 单边谱
P  = P2(1:floor(N/2)+1);
if length(P) > 2
    P(2:end-1) = 2 * P(2:end-1);   % 中间频率乘2补偿
end

% 幅值谱（注意：这里是幅值，不是能量）
Amp = sqrt(P);

% 频率轴（列向量）
f = (0:length(P)-1)' * (fs / N);

% ---------- 高频/低频划分 ----------
fc = fc_ratio * (fs/2);     % 高频阈值，单位 Hz
hf_idx = (f > fc);
lf_idx = ~hf_idx;

% ---------- 能量指标 ----------
freqMetrics.total_energy = sum(P);          % ≈ sum((v-mean(v)).^2)
freqMetrics.hf_energy    = sum(P(hf_idx));
freqMetrics.lf_energy    = sum(P(lf_idx));
freqMetrics.hf_energy_n  = freqMetrics.hf_energy / numel(v0);

% ---------- 按幅值阈值筛选“高抖动高频分量” ----------
if Amp_th > 0
    hf_strong_idx = hf_idx & (Amp > Amp_th);
else
    % 若阈值 <= 0，则认为所有高频都算“强高频”
    hf_strong_idx = hf_idx;
end

freqMetrics.hf_strong_energy_n = sum(P(hf_strong_idx)) / numel(v0);
hf_strong_energy = freqMetrics.hf_strong_energy_n;
% 保存谱数据
freqMetrics.f   = f;
freqMetrics.P   = P;
freqMetrics.Amp = Amp;

% ---------- 绘制频谱图 ----------
if doPlot
    figure;
    
    % 幅值谱
    % subplot(2,1,1);
    plot(f, Amp, 'LineWidth', 1.2); hold on;
    ylims = ylim;
    % 画出高频阈值 fc
    plot([fc fc], ylims, '--', 'LineWidth', 1);
    % 若设置了幅值阈值，则画水平线
    if Amp_th > 0
        plot([f(1) f(end)], [Amp_th Amp_th], ':', 'LineWidth', 1);
    end
    % 高抖动频率点做标记
    if any(hf_strong_idx)
        plot(f(hf_strong_idx), Amp(hf_strong_idx), 'o', 'MarkerSize', 4);
    end
    hold off;
    xlabel('频率 f (Hz)');
    ylabel('幅值');
    title('单边幅值谱');
    legend({'Amp(f)', 'f_c', 'Amp阈值', '高抖动频率'}, 'Location','best');
    grid on;
    % 
    % % 能量谱（可用对数坐标更清晰）
    % subplot(2,1,2);
    % plot(f, P, 'LineWidth', 1.2); hold on;
    % ylims2 = ylim;
    % plot([fc fc], ylims2, '--', 'LineWidth', 1);
    % hold off;
    % xlabel('频率 f (Hz)');
    % ylabel('能量谱 P(f)');
    % title('单边能量谱');
    % legend({'P(f)', 'f_c'}, 'Location','best');
    % grid on;
end

end
