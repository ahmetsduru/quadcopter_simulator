function ref_x_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve ref_x (5. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    ref_x = data{:, 5};      % 5. kolon: ref_x

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(ref_x)
        figure;
        plot(timestamp, ref_x, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('ref_x');
        title('ref_x vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end






