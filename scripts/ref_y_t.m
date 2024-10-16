function ref_y_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve ref_y (6. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    ref_y = data{:, 6};      % 6. kolon: ref_y

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(ref_y)
        figure;
        plot(timestamp, ref_y, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('ref_y');
        title('ref_y vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end






