function ref_z_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter', ',');

    % Timestamp (1. kolon) ve ref_z (7. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    ref_z = data{:, 7};      % 7. kolon: ref_z

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(ref_z)
        figure;
        plot(timestamp, ref_z, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('ref_z');
        title('ref_z vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end






