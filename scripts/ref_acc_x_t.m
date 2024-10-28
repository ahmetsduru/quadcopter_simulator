function ref_acc_x_t(log_folder)
    % Dosya yolunu belirleme (log klasöründen okuma)
    file_path = fullfile(log_folder, 'ref_pose.txt');

    % Veriyi okuma (readtable kullanarak daha esnek şekilde okuma)
    data = readtable(file_path, 'Delimiter',',');

    % Timestamp (1. kolon) ve ref_acc_x (8. kolon) verilerini ayırma
    timestamp = data{:, 1};  % 1. kolon: timestamp
    ref_acc_x = data{:, 8};      % 8. kolon: ref_acc_x

    % Eğer veriler doğruysa, grafiği çizelim
    if ~isempty(timestamp) && ~isempty(ref_acc_x)
        figure;
        plot(timestamp, ref_acc_x, 'LineWidth', 1.5);
        xlabel('Timestamp (s)');
        ylabel('ref_acc_x');
        title('ref_acc_x vs. Timestamp');
        grid on;
    else
        disp('Veriler boş olabilir, lütfen dosya formatını kontrol edin.');
    end
end